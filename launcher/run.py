#!/usr/bin/env python3
"""One personal, relocatable entry point for Humble native and Ubuntu24 host use."""
import argparse
import fcntl
import hashlib
import grp
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import urllib.request

ROOT=Path(__file__).resolve().parents[1]
STATE=ROOT/'.runtime';LOG=ROOT/'logs'
STATE.mkdir(exist_ok=True);LOG.mkdir(exist_ok=True)
# Unix domain sockets have a short pathname limit; USB paths can be long and Unicode.
SOCKET_DIR=Path('/tmp')/('yzbot-daemon-'+str(os.getuid())+'-'+hashlib.sha256(str(ROOT).encode()).hexdigest()[:10])
SOCKET_DIR.mkdir(exist_ok=True)
IMAGE='yzbot-personal-humble:20261019'
NAME='yzbot-personal-'+hashlib.sha256(str(ROOT).encode()).hexdigest()[:10]
OP=urllib.request.build_opener(urllib.request.ProxyHandler({}))
ENV=dict(os.environ,YZBOT_PROJECT=str(ROOT))


def native_alias():
    # Humble's CMake interface loader drops non-ASCII path fragments.
    # This temporary link contains no project data and is recreated after boot.
    alias=Path('/tmp')/('yzbot-native-'+str(os.getuid())+'-'+hashlib.sha256(str(ROOT).encode()).hexdigest()[:10])
    if alias.is_symlink() and alias.resolve()==ROOT:
        pass
    elif alias.exists() or alias.is_symlink():
        raise RuntimeError('临时运行路径被占用：'+str(alias))
    else:
        alias.symlink_to(ROOT,target_is_directory=True)
    ENV['YZBOT_NATIVE_ROOT']=str(alias)


def run(args,**kwargs):
    return subprocess.run([str(x) for x in args],check=True,env=ENV,**kwargs)


def api(path='/api/state'):
    with OP.open('http://127.0.0.1:8765'+path,timeout=3) as r:return json.load(r)


def output(args):
    p=subprocess.run([str(x) for x in args],env=ENV,text=True,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
    return p.returncode,p.stdout.strip()


def spawn(args,log):
    with (LOG/log).open('ab') as f:
        p=subprocess.Popen([str(x) for x in args],env=ENV,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    return p.pid


def daemon():
    missing=[cmd for cmd in ['docker','dockerd','containerd','sudo'] if not shutil.which(cmd)]
    if missing:
        if Path('/opt/ros/humble/setup.bash').exists():
            hint='本机已有 ROS Humble，直接运行 ./启动.sh（原生模式），无需安装 Docker。'
        else:
            hint='本机也没有 ROS Humble；请启动带 Humble 的 U 盘系统，或使用已装 Docker 的 Ubuntu。'
        raise RuntimeError('容器模式缺少 '+', '.join(missing)+'。'+hint)
    socket=SOCKET_DIR/'docker.sock';dc=['docker','-H','unix://'+str(socket)]
    if output(dc+['info'])[0]==0:return dc
    if subprocess.run(['sudo','-n','true'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
        run(['sudo','-v'])
    ctr=['sudo','-n','ctr','--address',SOCKET_DIR/'containerd.sock']
    if output(ctr+['version'])[0]:
        pid=spawn(['sudo','-n','containerd','--address',SOCKET_DIR/'containerd.sock','--root',STATE/'containerd-root','--state',STATE/'containerd-state'],'containerd.log')
        (STATE/'containerd.pid').write_text(str(pid))
        for _ in range(60):
            if output(ctr+['version'])[0]==0:break
            time.sleep(.5)
        else:raise RuntimeError('containerd 未就绪，查看 logs/containerd.log')
    pid=spawn(['sudo','-n','dockerd','--data-root='+str(STATE/'docker-data'),'--exec-root='+str(SOCKET_DIR/'docker-exec'),
        '--pidfile='+str(STATE/'dockerd.pid'),'--group='+grp.getgrgid(os.getgid()).gr_name,'--host=unix://'+str(socket),
        '--containerd='+str(SOCKET_DIR/'containerd.sock'),'--storage-driver=overlay2','--bridge=none','--iptables=false','--ip6tables=false','--userland-proxy=false'],'dockerd.log')
    (STATE/'docker-launch.pid').write_text(str(pid))
    for _ in range(120):
        if output(dc+['info'])[0]==0:return dc
        time.sleep(.5)
    raise RuntimeError('Docker 未就绪，查看 logs/dockerd.log')


def docker(ensure_image=True):
    dc=daemon()
    if ensure_image and output(dc+['image','inspect',IMAGE])[0]:
        archive=ROOT/'runtime/ros-humble-gazebo.tar.gz'
        if not archive.is_file():raise RuntimeError('缺少 runtime/ros-humble-gazebo.tar.gz，不能在 Ubuntu24 直接启动 Humble。')
        print('首次导入随包提供的 Humble/Gazebo 环境，可能需要几分钟。',flush=True)
        run(dc+['load','-i',archive])
    return dc


def is_running(dc):
    return output(dc+['inspect','-f','{{.State.Running}}',NAME])==(0,'true')


def runtime(dc):
    if is_running(dc):return
    subprocess.run(dc+['rm','-f',NAME],env=ENV,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    display=ENV.get('DISPLAY')
    if not display:raise RuntimeError('请在当前 Linux 图形桌面的终端启动，缺少 DISPLAY。')
    auth=Path(ENV.get('XAUTHORITY',f'/run/user/{os.getuid()}/gdm/Xauthority'))
    if not auth.is_file():raise RuntimeError('找不到桌面 Xauthority：'+str(auth))
    home=STATE/'container-home';home.mkdir(exist_ok=True)
    args=dc+['run','-d','--name',NAME,'--init','--network','host','--ipc','host',
        '--user',f'{os.getuid()}:{os.getgid()}',
        '-e','HOME=/opt/yzbot/.runtime/container-home','-e','DISPLAY='+display,
        '-e','XAUTHORITY=/tmp/yzbot.xauth','-e','LIBGL_ALWAYS_SOFTWARE=1',
        '-e','YZBOT_DOMAIN_ID='+ENV.get('YZBOT_DOMAIN_ID','91'),
        '-e','YZBOT_TASK_MODE='+ENV.get('YZBOT_TASK_MODE','exam'),
        '-e','YZBOT_OFFICIAL_ADAPTER='+ENV.get('YZBOT_OFFICIAL_ADAPTER','qwen_exam_adapter:solve_task'),
        '-e','YZBOT_PRACTICE_LLM_URL='+ENV.get('YZBOT_PRACTICE_LLM_URL','http://127.0.0.1:8081'),
        '-e','YZBOT_LOCALIZATION=slam','-e','LP_NUM_THREADS=4',
        '-v',str(ROOT)+':/opt/yzbot','-v',str(auth)+':/tmp/yzbot.xauth:ro',
        '-v','/tmp/.X11-unix:/tmp/.X11-unix','-w','/opt/yzbot',
        '--entrypoint','/bin/bash',IMAGE,'-c','exec sleep infinity']
    run(args,stdout=subprocess.DEVNULL)


def source_digest():
    h=hashlib.sha256()
    for folder in ['ros_ws/src','native','tools']:
        for path in sorted((ROOT/folder).rglob('*')):
            if not path.is_file() or any(x in path.parts for x in ('build','install','log','__pycache__')):continue
            if path.name!='cmake' and path.suffix not in ('.py','.sh','.xacro','.xml','.txt','.cpp','.h','.hpp','.yaml'):continue
            h.update(str(path.relative_to(ROOT)).encode());h.update(path.read_bytes())
    for path in sorted((ROOT/'ros_ws').glob('*.py')):
        h.update(path.name.encode());h.update(path.read_bytes())
    h.update((ROOT/'ros_ws/scene_config.json').read_bytes())
    return h.hexdigest()


def build(mode,dc,force=False):
    digest=source_digest();fingerprint=mode+':'+str(ROOT)+':'+digest
    stamp=STATE/'build-fingerprint.txt'
    if not force and stamp.exists() and stamp.read_text()==fingerprint and (ROOT/'ros_ws/install/local_setup.bash').is_file():return
    print('构建本目录中的机器人程序（首次约几分钟）…',flush=True)
    # Build products contain absolute prefixes. Move generated products aside when switching native/container roots.
    if stamp.exists() and not stamp.read_text().startswith(mode+':'+str(ROOT)+':'):
        backup=STATE/('previous-build-'+str(int(time.time())));backup.mkdir()
        for rel in ['ros_ws/build','ros_ws/install','ros_ws/log','ros_ws/src/yzbot/Move_Obs/build']:
            p=ROOT/rel
            if p.exists():
                dst=backup/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.move(str(p),str(dst))
    command=['bash',ROOT/'native/build.sh'] if mode=='native' else dc+['exec',NAME,'bash','/opt/yzbot/native/build.sh']
    with (LOG/'build.log').open('w') as f:
        p=subprocess.run([str(x) for x in command],env=ENV,stdout=f,stderr=subprocess.STDOUT)
    if p.returncode:
        print((LOG/'build.log').read_text()[-7000:],file=sys.stderr);raise RuntimeError('构建失败，见 logs/build.log')
    stamp.write_text(mode+':'+str(ROOT)+':'+source_digest())


def stop():
    statefile=STATE/'run-mode.json'
    mode=json.loads(statefile.read_text()).get('mode') if statefile.exists() else ('native' if Path('/opt/ros/humble/setup.bash').exists() else 'container')
    if mode=='container':
        if shutil.which('docker'):
            dc=['docker','-H','unix://'+str(SOCKET_DIR/'docker.sock')]
            if output(dc+['info'])[0]==0 and is_running(dc):
                subprocess.run(dc+['exec',NAME,'bash','/opt/yzbot/native/stop.sh'],env=ENV)
                run(dc+['stop','-t','8',NAME],stdout=subprocess.DEVNULL)
    elif Path('/opt/ros/humble/setup.bash').exists():
        native_alias()
        run(['bash',ROOT/'native/stop.sh'])
    sys.path.insert(0,str(ROOT/'native'));from process_guard import belongs
    f=LOG/'host-qwen.pid'
    if f.exists() and belongs(f.read_text().strip(),ROOT):os.killpg(int(f.read_text()),signal.SIGTERM)
    # Stop only this folder's private daemons. No system Docker or other projects are touched.
    for name in ['dockerd.pid','containerd.pid']:
        f=STATE/name
        if f.exists():
            try:
                pid=int(f.read_text());command=(Path('/proc')/str(pid)/'cmdline').read_bytes()
                if str(STATE).encode() in command:subprocess.run(['sudo','-n','kill','-TERM',str(pid)],check=False)
            except (OSError,ValueError):pass
    print('本目录的运行程序已停止，源码和日志保留。',flush=True)


def model():
    if ENV.get('YZBOT_OFFICIAL_ADAPTER','qwen_exam_adapter:solve_task')!='qwen_exam_adapter:solve_task':return
    run(['bash',ROOT/'launcher/start-model.sh'])


def open_panel():
    if shutil.which('xdg-open'):
        subprocess.Popen(['xdg-open','http://127.0.0.1:8765/'],env=ENV,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['start','stop','build','status'],nargs='?',default='start');p.add_argument('--native',action='store_true');p.add_argument('--container',action='store_true');p.add_argument('--no-open',action='store_true');a=p.parse_args()
    lock=(STATE/'launcher.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise RuntimeError('已有启动/停止操作在进行，请查看该终端。')
    if a.action in ('start','build'):(LOG/'startup-error.txt').unlink(missing_ok=True)
    if a.action=='stop':stop();return
    if a.action=='status':print(json.dumps(api(),ensure_ascii=False,indent=2));return
    if a.native and a.container:raise RuntimeError('--native 和 --container 不能同时使用。')
    humble=Path('/opt/ros/humble/setup.bash').is_file()
    docker_ready=all(shutil.which(x) for x in ['docker','dockerd','containerd','sudo'])
    if a.native:
        mode='native'
    elif a.container:
        mode='container'
    elif humble:
        mode='native'
    elif docker_ready:
        mode='container'
    else:
        raise RuntimeError('没有找到可运行环境：本机没有 ROS Humble，也没有 Docker。U 盘系统已带 ROS Humble/Gazebo，请在 U 盘 Ubuntu 桌面里运行本目录的 ./启动.sh。')
    if mode=='native' and not humble:raise RuntimeError('当前系统未安装 ROS Humble；请使用 --container（需系统已安装 Docker）。')
    if mode=='native':native_alias()
    old_state=STATE/'run-mode.json'
    old_mode=json.loads(old_state.read_text()).get('mode') if old_state.exists() else None
    if old_mode and old_mode!=mode:
        print('检测到上次运行方式不同，先停止本目录旧实例…',flush=True)
        stop()
    print('运行方式：'+('本机已有 ROS Humble（不下载、不用 Docker）' if mode=='native' else '本目录隔离的 Ubuntu22.04/Humble 容器'),flush=True)
    dc=docker() if mode=='container' else None
    if dc:runtime(dc)
    (STATE/'run-mode.json').write_text(json.dumps(dict(mode=mode)))
    stamp=STATE/'build-fingerprint.txt'
    expected=mode+':'+str(ROOT)+':'+source_digest()
    changed=not stamp.exists() or stamp.read_text()!=expected
    if changed or a.action=='build':
        # Stop our ROS services before changing loaded shared libraries or runtime code.
        if dc:subprocess.run(dc+['exec',NAME,'bash','/opt/yzbot/native/stop.sh'],env=ENV,stdout=subprocess.DEVNULL)
        else:subprocess.run(['bash',str(ROOT/'native/stop.sh')],env=ENV,stdout=subprocess.DEVNULL)
    build(mode,dc,force=a.action=='build')
    if a.action=='build':return
    model()
    command=['bash',ROOT/'native/start.sh'] if mode=='native' else dc+['exec',NAME,'bash','/opt/yzbot/native/start.sh']
    result=subprocess.run([str(x) for x in command],env=ENV)
    if result.returncode:raise RuntimeError('启动没有全部就绪，见 logs/startup-error.txt 和 logs/*.log。')
    # Camera, TF and map topics can lag briefly after the ROS launch process exits.
    # Give the complete health check a bounded grace period before reporting failure.
    state=api()
    deadline=time.monotonic()+15
    while not state.get('healthy') and time.monotonic()<deadline:
        time.sleep(.5)
        state=api()
    if not state.get('healthy'):
        issue='、'.join(state.get('health_issues') or ['状态未就绪'])
        print('\n网页服务已启动，但机器人还未全部就绪：'+issue,flush=True)
        print('已尝试打开诊断面板：http://127.0.0.1:8765/\n日志位置：'+str(LOG),flush=True)
        if not a.no_open:open_panel()
        raise RuntimeError('机器人健康检查未通过：'+issue)
    print('\n全部启动就绪.浏览器：http://127.0.0.1:8765/\n先确认规则与顺序，再点击“开始考试”。\n关闭：./停止.sh',flush=True)
    if not a.no_open:open_panel()


if __name__=='__main__':
    try:main()
    except (RuntimeError,subprocess.CalledProcessError,urllib.error.URLError,OSError) as error:
        (LOG/'startup-error.txt').write_text(str(error)+'\n')
        print('\n启动/操作失败：'+str(error),file=sys.stderr);sys.exit(1)
