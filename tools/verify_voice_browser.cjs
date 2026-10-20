const fs=require('fs');const path=require('path');const WebSocket=require('ws');
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
(async()=>{
 const target=await(await fetch('http://127.0.0.1:9227/json/new?about:blank',{method:'PUT'})).json();
 const socket=new WebSocket(target.webSocketDebuggerUrl);await new Promise(r=>socket.on('open',r));
 let id=0;const pending=new Map(),external=[];
 socket.on('message',raw=>{const m=JSON.parse(raw);if(m.id&&pending.has(m.id)){const p=pending.get(m.id);pending.delete(m.id);m.error?p.reject(m.error):p.resolve(m.result)}
  if(m.method==='Fetch.requestPaused'){const p=m.params,url=p.request.url;if(/^(http|ws)s?:/.test(url)&&!url.startsWith('http://127.0.0.1:8765/')){external.push(url);call('Fetch.failRequest',{requestId:p.requestId,errorReason:'InternetDisconnected'}).catch(()=>{})}else call('Fetch.continueRequest',{requestId:p.requestId}).catch(()=>{})}
 });
 function call(method,params={}){return new Promise((resolve,reject)=>{const k=++id;pending.set(k,{resolve,reject});socket.send(JSON.stringify({id:k,method,params}));setTimeout(()=>{if(pending.has(k)){pending.delete(k);reject(Error('timeout '+method))}},25000).unref()})}
 async function evaluate(expression){const r=await call('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value}
 const report={scope:'Browser audio decoding/playback and portable copied assets; no physical listening or USB boot test',externalNetworkBlocked:true};
 try{
 await call('Page.enable');await call('Runtime.enable');await call('Fetch.enable',{patterns:[{urlPattern:'*'}]});
 await call('Emulation.setDeviceMetricsOverride',{width:1366,height:768,deviceScaleFactor:1,mobile:false});
 await call('Page.navigate',{url:'http://127.0.0.1:8765/'});await sleep(2200);
 const rect=await evaluate(`(()=>{const r=document.getElementById('voiceToggle').getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2}})()`);
 await call('Input.dispatchMouseEvent',{type:'mousePressed',button:'left',clickCount:1,...rect});
 await call('Input.dispatchMouseEvent',{type:'mouseReleased',button:'left',clickCount:1,...rect});
 await sleep(1800);
 report.liveEnable=await evaluate(`({enabled:voice.enabled,context:voice.context?.state,playing:!!voice.source,hint:document.getElementById('voiceHint').textContent,caption:document.getElementById('voiceText').textContent,time:voice.context?.currentTime})`);
 if(!report.liveEnable.enabled||report.liveEnable.context!=='running'||!report.liveEnable.playing)throw Error('Real click did not start audio');
 await sleep(300);
 report.audioClockAdvances=await evaluate(`voice.context.currentTime>${report.liveEnable.time+.15}`);
 report.decoded=await evaluate(`(async()=>{const m=await voice.loadManifest(),out=[];for(const [id,clip] of Object.entries(m.clips)){const b=await voice.buffer(id),v=b.getChannelData(0);let sum=0,peak=0;for(let i=0;i<v.length;i++){sum+=v[i]*v[i];peak=Math.max(peak,Math.abs(v[i]))}out.push({id,duration:b.duration,rate:b.sampleRate,channels:b.numberOfChannels,rms:Math.sqrt(sum/v.length),peak})}return out})()`);
 if(report.decoded.length!==29||report.decoded.some(x=>x.duration<.3||x.rms<.001))throw Error('Audio decode/energy failed');
 report.lifecycle=await evaluate(`(async()=>{const a=await voice.say('grasp_blue','check:grasp'),old=voice.source,duplicate=await voice.say('grasp_blue','check:grasp'),b=await voice.say('carry_blue_b','check:carry');const replaced=old!==voice.source&&old.onended===null;const original=voice.buffer.bind(voice),slow=await original('search');let finish;voice.buffer=id=>id==='search'?new Promise(r=>finish=r):original(id);const stale=voice.say('search','check:slow');const fresh=await voice.say('place','check:fresh'),latest=voice.source;finish(slow);const staleResult=await stale;const latestPreserved=voice.source===latest;let finishOff;voice.buffer=()=>new Promise(r=>finishOff=r);const stopping=voice.say('search','check:disable');voice.disable();finishOff(slow);const stoppedResult=await stopping;voice.buffer=original;return {first:a,duplicateSkipped:duplicate===false,replaced,carry:b,newestOnly:fresh&&!staleResult&&latestPreserved,disabledCancels:!stoppedResult&&!voice.source&&!voice.enabled,captionPreserved:!!document.getElementById('voiceText').textContent}})()`);
 if(Object.values(report.lifecycle).some(v=>v!==true))throw Error('Lifecycle failed');
 report.phaseMapping=await evaluate(`['red','blue','target'].flatMap(c=>['A','B','C'].map(z=>{const [id,text]=voiceMessage({phase:'搬运',status:c+' 前往 '+z+' 区',placed:2,total:5});return {id,valid:!!voice.manifest.clips[id],chinese:!/\\ufffd/.test(text)}}))`);
 if(report.phaseMapping.some(x=>!x.valid||!x.chinese))throw Error('Caption/phase mapping failed');
 report.missingPack=await evaluate(`(async()=>{let hint='';const v=new CompetitionVoice(t=>hint=t,'/api/no-voice/');const ok=await v.enable();return {failedClearly:!ok&&!v.enabled,captionOnly:hint.includes('语音未就绪')}})()`);
 if(!report.missingPack.failedClearly||!report.missingPack.captionOnly)throw Error('Missing pack not handled');
 const copy=path.resolve('.runtime/portable-voice-check/带空格 的移动目录/项目');
 fs.mkdirSync(path.join(copy,'assets'),{recursive:true});fs.cpSync('assets/voice',path.join(copy,'assets/voice'),{recursive:true});fs.copyFileSync('语音试听.sh',path.join(copy,'语音试听.sh'));
 await call('Page.navigate',{url:new URL('file://'+path.join(copy,'assets/voice/试听.html')).href});await sleep(1500);
 const ar=await evaluate(`(()=>{const a=document.querySelector('audio'),r=a.getBoundingClientRect();return {x:r.x+18,y:r.y+r.height/2}})()`);
 await call('Input.dispatchMouseEvent',{type:'mousePressed',button:'left',clickCount:1,...ar});await call('Input.dispatchMouseEvent',{type:'mouseReleased',button:'left',clickCount:1,...ar});await sleep(800);
 report.portableFilePreview=await evaluate(`(()=>{const a=document.querySelector('audio');return {url:location.href,ready:a.readyState,playing:!a.paused,time:a.currentTime,duration:a.duration,error:a.error?.code||null}})()`);
 if(!report.portableFilePreview.playing||report.portableFilePreview.time<.1||report.portableFilePreview.error)throw Error('Copied offline preview did not play');
 report.externalRequests=external;
 if(external.length)throw Error('Unexpected external request');
 report.passed=true;
 }finally{fs.writeFileSync('verification/voice-checks.json',JSON.stringify(report,null,2));await fetch('http://127.0.0.1:9227/json/close/'+target.id);socket.close()}
 console.log(JSON.stringify(report,null,2));
})().catch(e=>{console.error(e);process.exitCode=1});
