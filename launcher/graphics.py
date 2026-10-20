"""Reuse the host's installed graphics driver without downloading a driver."""
import grp,hashlib,json,os
from pathlib import Path


def container_graphics(software=False):
    options=[];libraries=[];devices=[]
    nvidia=not software and Path('/dev/nvidia0').exists() and Path('/usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.0').exists()
    if nvidia:
        patterns=['libGLX_nvidia.so*','libEGL_nvidia.so*','libnvidia-glcore.so*','libnvidia-tls.so*',
                  'libnvidia-glsi.so*','libnvidia-gpucomp.so*','libnvidia-eglcore.so*']
        for pattern in patterns:
            for path in sorted(Path('/usr/lib/x86_64-linux-gnu').glob(pattern)):
                source=path.resolve();target='/usr/local/nvidia/lib64/'+path.name
                options+=['-v',str(source)+':'+target+':ro']
                libraries.append((str(source),source.stat().st_size,source.stat().st_mtime_ns,target))
        options+=['-e','YZBOT_GRAPHICS_LIBRARY_PATH=/usr/local/nvidia/lib64',
                  '-e','LD_LIBRARY_PATH=/usr/local/nvidia/lib64','-e','__GLX_VENDOR_LIBRARY_NAME=nvidia',
                  '-e','__GL_SYNC_TO_VBLANK=0']
        for name in ['nvidia0','nvidiactl','nvidia-modeset','nvidia-uvm']:
            path=Path('/dev')/name
            if path.exists():devices.append(str(path))
    if not software and Path('/dev/dri').exists():devices.append('/dev/dri')
    for device in devices:options+=['--device',device]
    for group in ['video','render']:
        try:options+=['--group-add',str(grp.getgrnam(group).gr_gid)]
        except KeyError:pass
    mode='nvidia-host' if nvidia else 'mesa-device' if devices else 'software'
    if mode=='software':options+=['-e','LIBGL_ALWAYS_SOFTWARE=1']
    signature=hashlib.sha256(json.dumps([mode,devices,libraries]).encode()).hexdigest()[:16]
    return options,dict(mode=mode,signature=signature,devices=devices,host_driver_libraries=len(libraries))
