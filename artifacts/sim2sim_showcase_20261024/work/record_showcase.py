#!/usr/bin/env python3
"""Run the repository's actual fused policy/physics runtime and capture native video."""
import argparse, json, os, pathlib, signal, subprocess, time
import numpy as np
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32, Float32MultiArray
from geometry_msgs.msg import Twist

ROOT = pathlib.Path(__file__).resolve().parents[3]
OUT = pathlib.Path(__file__).resolve().parents[1]
WORK = OUT / 'work'
CFG = ROOT / 'src/omnimorph_rl_controller/rl_master/config'
SIM = ROOT / 'src/omnimorph_sim2sim/mujoco_sim2sim'
SPECS = {
 'amp': ('jc01', 1, 'jc01_amp_full_body_sim2sim.yaml', '/home/edify/Code/jc01-model/scene_jingchu01.xml'),
 'taichi': ('jc01', 8, 'jc01_amp_full_body_sim2sim.yaml', '/home/edify/Code/jc01-model/scene_jingchu01.xml'),
 'dance': ('jc01', 7, 'jc01_amp_full_body_sim2sim.yaml', '/home/edify/Code/jc01-model/scene_jingchu01.xml'),
 'omnixtreme': ('unitree_g1', 2, 'unitree_g1_beyondmimic_sim2sim.yaml', '/home/edify/Code/OmniXtreme/robots/g1/no_hand.xml'),
}

def prepare(key, tag, args):
    robot, mode, preset, model = SPECS[key]
    root = yaml.safe_load((CFG / f'rl_cfg_{robot}.yaml').read_text())
    root['omnimorph_root_dir'] = str(CFG.parent)
    root['humanoid_rl_root_dir'] = str(CFG.parent)
    root['path_variables'].update(robot_assets_dir='/home/edify/Code/jc01-model',
        jc01_pinocchio_urdf='/home/edify/Code/jc01-model/JC01-URDF.urdf', jc01_scene_xml=model,
        g1_pinocchio_urdf='/home/edify/Code/unitree_g1_model/g1_29dof.urdf', g1_scene_xml=model)
    root['logging']['output_dir'] = str(WORK / 'runtime_logs')
    for name, path in root['config_files'].items():
        root['config_files'][name] = str(CFG / path)
    # Match the dance profile's declared name to the existing dance ONNX.
    if key == 'dance':
        name = 'beyondmimic_jc01_dance_full_body_wo_state_estimation'
        profile = yaml.safe_load((CFG / 'profiles' / (name+'.yaml')).read_text())
        profile[name]['policy_file'] = str(CFG.parent / 'policies/beyondmimic_jc01_dance_wo_state_estimation.onnx')
        profile[name]['observation_manifest_file'] = str(CFG / profile[name]['observation_manifest_file'])
        path = WORK / f'{tag}_profile.yaml'
        path.write_text(yaml.safe_dump(profile, sort_keys=False)); root['config_files'][name] = str(path)
    selected = next(item for item in root['deploy_mode_profiles'] if item['mode_id'] == mode)
    root['deploy_mode_profiles'] = [selected]
    root['config_files'] = {selected['config_section']: root['config_files'][selected['config_section']]}
    root_path = WORK / f'{tag}_root.yaml'
    root_path.write_text(yaml.safe_dump(root, sort_keys=False))
    params = yaml.safe_load((SIM / 'config' / preset).read_text())
    p = params['mujoco_sim_bridge']['ros__parameters']
    p.update(model_path=model, rl_cfg_path=str(root_path), startup_mode_id=mode,
        enable_viewer=False, enable_python_viewer_stream=True, viewer_fps=60.0,
        enable_python_viewer_inspector=False, enable_com_support_visualization=False,
        enable_dcm_capture_visualization=False, enable_video_recording=False,
        video_output_dir=str(WORK), video_output_name=f'{tag}_raw.mp4', video_fps=30.0,
        video_width=1280, video_height=720, video_follow_robot=True,
        video_follow_distance=3.4 if key == 'omnixtreme' else 3.3,
        video_follow_azimuth=135.0, video_follow_elevation=-15.0,
        video_follow_lookat_offset=[0.0,0.0,0.0],
        post_release_settle_ticks=0, sim_sync_running_start_to_reference=(key != 'amp'))
    if args.no_sync:
        p['sim_sync_running_start_to_reference'] = False
    params_path = WORK / f'{tag}_bridge.yaml'
    params_path.write_text(yaml.safe_dump(params, sort_keys=False))
    return mode, params_path

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('key',choices=SPECS); ap.add_argument('--tag'); ap.add_argument('--duration',type=float,default=16.0); ap.add_argument('--no-sync',action='store_true'); ap.add_argument('--walk-speed',type=float,default=0.6); ap.add_argument('--run-speed',type=float,default=1.8)
    args=ap.parse_args(); tag=args.tag or args.key
    mode, params_path=prepare(args.key,tag,args)
    rclpy.init(); node=rclpy.create_node('showcase_director')
    mode_pub=node.create_publisher(Int32,'/omnimorph/rl/mode_control',10)
    teleop=node.create_publisher(Twist,'/omnimorph/rl/teleop',qos_profile_sensor_data)
    samples=[]; latest=[None]
    def receive(msg):
        d=np.asarray(msg.data,dtype=np.float32)
        latest[0]=d; samples.append(d.copy())
    sub=node.create_subscription(Float32MultiArray,'/omnimorph/sim2sim/mujoco_viewer_frame',receive,qos_profile_sensor_data)
    logpath=WORK/f'{tag}.log'
    cmd=[str(ROOT/'install/mujoco_sim2sim/lib/mujoco_sim2sim/mujoco_sim_bridge'),'--ros-args','--params-file',str(params_path)]
    print('Launching', tag, flush=True)
    meta={'key':args.key,'tag':tag,'mode':mode,'command':cmd,'duration_requested':args.duration}
    with logpath.open('w') as log:
        proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+max(120,args.duration*4); sent=False; start=None; start_sent=0; last_pub=0
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=0.005)
                if proc.poll() is not None:
                    raise RuntimeError(f'Runtime exited {proc.returncode}; see {logpath}')
                text=logpath.read_text(errors='replace')
                now=time.monotonic()
                if not sent and 'lifecycle -> HOLD' in text and latest[0] is not None and mode_pub.get_subscription_count():
                    mode_pub.publish(Int32(data=1000+mode)); sent=True; start_sent=now
                    print('START',tag,'sim_t=',float(latest[0][5]),flush=True)
                if sent and start is None and 'lifecycle -> RUNNING' in text:
                    start=float(latest[0][5]); meta['running_sim_time']=start
                    print('RUNNING',tag,'sim_t=',start,flush=True)
                if now-last_pub>0.025:
                    msg=Twist()
                    if args.key=='amp' and start is not None:
                        elapsed=float(latest[0][5])-start
                        # Demonstrate steady walking followed by gradual acceleration.
                        msg.linear.x=args.walk_speed if elapsed<5 else min(args.run_speed,args.walk_speed+(elapsed-5)*0.4)
                    teleop.publish(msg); last_pub=now
                if start is not None and float(latest[0][5])-start>=args.duration:
                    meta['ending_sim_time']=float(latest[0][5]); break
            else:
                raise TimeoutError('Runtime did not complete requested simulation duration')
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try: proc.wait(timeout=55)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM); proc.wait(timeout=10)
            node.destroy_node(); rclpy.shutdown()
            if samples:
                np.savez_compressed(WORK/f'{tag}_states.npz',frames=np.stack(samples))
            meta['exit_code']=proc.returncode
            (WORK/f'{tag}_metadata.json').write_text(json.dumps(meta,indent=2))
    if 'ending_sim_time' not in meta:
        raise RuntimeError('Capture incomplete')
    a=np.stack(samples); running=a[(a[:,5]>=meta['running_sim_time']) & (a[:,5]<=meta['ending_sim_time'])]
    q=running[:,8:15]
    print(json.dumps({'tag':tag,'samples':len(running),'base_z_range':[float(q[:,2].min()),float(q[:,2].max())], 'base_xy_end':q[-1,:2].tolist(),'exit_code':proc.returncode}),flush=True)

if __name__=='__main__': main()
