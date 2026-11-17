#!/usr/bin/env python3
"""Render authentic recorded sim2sim states with a robot-following camera."""
import argparse, json, pathlib, subprocess
import numpy as np
import mujoco
import yaml

HERE=pathlib.Path(__file__).resolve().parent

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('tag'); ap.add_argument('--output'); ap.add_argument('--start',type=float,default=0.5); ap.add_argument('--duration',type=float,default=15.0); ap.add_argument('--distance',type=float); ap.add_argument('--azimuth',type=float,default=45.0); ap.add_argument('--width',type=int,default=1920); ap.add_argument('--height',type=int,default=1080)
    a=ap.parse_args()
    meta=json.loads((HERE/f'{a.tag}_metadata.json').read_text())
    p=yaml.safe_load((HERE/f'{a.tag}_bridge.yaml').read_text())['mujoco_sim_bridge']['ros__parameters']
    frames=np.load(HERE/f'{a.tag}_states.npz')['frames']
    # A message can repeat a physical state; keep only monotonic timestamps.
    frames=frames[np.r_[True,np.diff(frames[:,5])>0]]
    t=frames[:,5].astype(float); nq,nv,nu=frames[0,2:5].astype(int)
    start=meta['running_sim_time']+a.start
    times=start+np.arange(round(a.duration*30))/30
    if times[-1]>t[-1]: raise ValueError('Recorded simulation is shorter than requested video')
    model=mujoco.MjModel.from_xml_path(p['model_path']); data=mujoco.MjData(model)
    assert model.nq==nq
    model.vis.global_.offwidth=a.width; model.vis.global_.offheight=a.height
    model.vis.quality.shadowsize=1024; model.vis.quality.offsamples=0
    renderer=mujoco.Renderer(model,a.height,a.width)
    camera=mujoco.MjvCamera(); mujoco.mjv_defaultCamera(camera)
    camera.type=mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance=a.distance or p['video_follow_distance']; camera.azimuth=a.azimuth; camera.elevation=-12
    option=mujoco.MjvOption(); mujoco.mjv_defaultOption(option)
    # Display visual meshes, suppress collision-only duplicates and sensor markers.
    option.geomgroup[3:]=0
    output=pathlib.Path(a.output) if a.output else HERE.parent/f'{a.tag}.mp4'
    ff=subprocess.Popen(['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{a.width}x{a.height}','-r','30','-i','-','-an','-c:v','libx264','-preset','medium','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE)
    body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,p['base_body_name'])
    pngs=[]
    try:
        for k,stamp in enumerate(times):
            j=int(np.clip(np.searchsorted(t,stamp),1,len(t)-1)); l=j-1
            w=np.clip((stamp-t[l])/(t[j]-t[l]),0,1)
            state=frames[l]*(1-w)+frames[j]*w
            data.qpos[:]=state[8:8+nq]
            data.qpos[3:7]/=np.linalg.norm(data.qpos[3:7])
            data.qvel[:]=state[8+nq:8+nq+nv]
            data.ctrl[:]=state[8+nq+nv:8+nq+nv+nu]
            data.time=stamp; mujoco.mj_forward(model,data)
            camera.lookat[:]=data.xpos[body]
            # The camera follows translation while retaining a stable world heading.
            renderer.update_scene(data,camera=camera,scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
            rgb=renderer.render(); ff.stdin.write(rgb.tobytes())
            if k in [0,90,180,270,360,449]:
                from PIL import Image
                path=HERE/f'{a.tag}_frame_{k:04d}.jpg'
                Image.fromarray(rgb).resize((640,360)).save(path,quality=90); pngs.append(str(path))
            if k%90==0: print(a.tag,'render',k,'/',len(times),flush=True)
    finally:
        ff.stdin.close(); code=ff.wait(); renderer.close()
    if code: raise RuntimeError(f'ffmpeg failed: {code}')
    meta['render']={'path':str(output),'duration':a.duration,'fps':30,'size':[a.width,a.height],'first_sim_time':start,'camera':{'follow_robot':True,'distance':camera.distance,'azimuth':camera.azimuth,'elevation':camera.elevation},'source':'actual fused sim2sim qpos/qvel/ctrl telemetry, interpolated to 30 fps'}
    (HERE/f'{a.tag}_metadata.json').write_text(json.dumps(meta,indent=2))
    from PIL import Image,ImageOps,ImageDraw
    sheet=Image.new('RGB',(1280,3*390),(24,28,34)); draw=ImageDraw.Draw(sheet)
    for i,path in enumerate(pngs):
        sheet.paste(Image.open(path),(i%2*640,i//2*390)); draw.text((i%2*640+12,i//2*390+365),f'{a.tag} | {i*3:.0f} s',fill='white')
    sheet.save(HERE/f'{a.tag}_contact_sheet.jpg',quality=90)
    print('Saved',output,flush=True)

if __name__=='__main__': main()
