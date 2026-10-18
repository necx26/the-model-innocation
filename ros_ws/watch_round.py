import json,time,urllib.request
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
last=None
end=time.monotonic()+330
while time.monotonic()<end:
 try:
  s=json.load(opener.open('http://127.0.0.1:8765/api/state',timeout=3))
  value=(s['phase'],s['status'],s['picked'],s['placed'])
  if value!=last:
   print(json.dumps({k:s[k] for k in ['phase','status','elapsed','picked','placed','robot']},ensure_ascii=False),flush=True);last=value
  if not s['running'] and s['phase'] not in ['就绪','大模型解题']:break
 except Exception as e:print(str(e),flush=True)
 time.sleep(2)
