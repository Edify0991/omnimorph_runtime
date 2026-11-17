import argparse,json,pathlib
import numpy as np,mujoco,yaml
from PIL import Image,ImageDraw
ap=argparse.ArgumentParser(); ap.add_argument('tag'); ap.add_argument('--start',type=float,default=0); ap.add_argument('--duration',type=float,default=30); a=ap.parse_args()
p=pathlib.Path(__file__).resolve().parent
meta=json.loads((p/f'{a.tag}_metadata.json').read_text()); par=yaml.safe_load((p/f'{a.tag}_bridge.yaml').read_text())['mujoco_sim_bridge']['ros__parameters']
f=np.load(p/f'{a.tag}_states.npz')['frames']; nq,nv,nu=f[0,2:5].astype(int)
m=mujoco.MjModel.from_xml_path(par['model_path']); d=mujoco.MjData(m); m.vis.quality.shadowsize=1024;m.vis.quality.offsamples=0
r=mujoco.Renderer(m,360,640); c=mujoco.MjvCamera(); mujoco.mjv_defaultCamera(c);c.distance=3.3;c.azimuth=45;c.elevation=-12
opt=mujoco.MjvOption();mujoco.mjv_defaultOption(opt);opt.geomgroup[3:]=0
body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,par['base_body_name'])
sheet=Image.new('RGB',(1920,4*390),(24,28,34)); draw=ImageDraw.Draw(sheet)
for k,dt in enumerate(np.linspace(a.start,a.start+a.duration,12)):
 stamp=meta['running_sim_time']+dt;j=np.argmin(abs(f[:,5]-stamp));d.qpos[:]=f[j,8:8+nq];d.qvel[:]=f[j,8+nq:8+nq+nv];mujoco.mj_forward(m,d)
 c.lookat[:]=d.xpos[body];r.update_scene(d,camera=c,scene_option=opt);r.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
 im=Image.fromarray(r.render()); sheet.paste(im,(k%3*640,k//3*390));draw.text((k%3*640+12,k//3*390+365),f'{a.tag} | {dt:.1f}s',fill='white')
r.close();sheet.save(p/f'{a.tag}_preview.jpg',quality=90);print('Saved preview',flush=True)
