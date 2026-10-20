/* Project-local Chinese voice playback; no speechSynthesis or external requests. */
'use strict';
(function (root) {
  const ownerKey = 'yzbot:competition-voice-owner:v1';
  class CompetitionVoice {
    constructor(onStatus, base = '/api/voice/') {
      this.onStatus = onStatus;
      this.base = base;
      this.enabled = false;
      this.generation = 0;
      this.lastKey = null;
      this.buffers = new Map();
      this.source = null;
      this.context = null;
      this.manifest = null;
      this.manifestPromise = null;
      this.owner = root.crypto?.randomUUID?.() || Date.now()+'-'+Math.random();
      this.ownerStorage = true;
      try { root.localStorage.getItem(ownerKey); } catch (_) { this.ownerStorage = false; }
      this.channel = typeof root.BroadcastChannel === 'function'
        ? new root.BroadcastChannel(ownerKey) : null;
      if (this.channel) this.channel.onmessage = event => {
        if (event.data?.owner && event.data.owner !== this.owner && this.enabled &&
            (!this.ownerStorage || !this.ownsAudio()))
          this.disable('另一比赛页面已接管语音');
      };
      root.addEventListener('storage', event => {
        if (event.key === ownerKey && event.newValue && event.newValue !== this.owner &&
            this.enabled && !this.ownsAudio())
          this.disable('另一比赛页面已接管语音');
      });
      root.addEventListener('pagehide', () => this.disable());
      this.cancelLegacySpeech();
    }
    status(text, speaking = false) { this.onStatus(text, speaking, this.enabled); }
    cancelLegacySpeech() {
      // Cancel a previous page's queued native utterances; never create native speech.
      try { root.speechSynthesis?.cancel(); } catch (_) { /* Not supported. */ }
    }
    claim() {
      if (this.ownerStorage) {
        try { root.localStorage.setItem(ownerKey, this.owner); }
        catch (_) { this.ownerStorage = false; }
      }
      this.channel?.postMessage({owner: this.owner});
    }
    ownsAudio() {
      if (!this.ownerStorage) return true;
      try { return root.localStorage.getItem(ownerKey) === this.owner; }
      catch (_) { return true; }
    }
    async loadManifest() {
      if (!this.manifestPromise) {
        this.manifestPromise = fetch(this.base + 'manifest.json', {signal: AbortSignal.timeout(6000)})
          .then(r => { if (!r.ok) throw Error('语音包不可用'); return r.json(); })
          .then(data => {
            if (data.language !== 'zh-CN' || data.gender !== 'female' || !data.clips?.enabled)
              throw Error('中文女声包不完整');
            this.manifest = data;
            return data;
          }).catch(error => { this.manifestPromise = null; throw error; });
      }
      return this.manifestPromise;
    }
    async buffer(id) {
      if (this.buffers.has(id)) return this.buffers.get(id);
      const promise = (async () => {
        const manifest = await this.loadManifest(), clip = manifest.clips[id];
        if (!clip || !/^[a-z][a-z0-9_]*\.mp3$/.test(clip.file)) throw Error('缺少中文语音');
        const r = await fetch(this.base + clip.file, {signal: AbortSignal.timeout(6000)});
        if (!r.ok) throw Error('语音文件读取失败');
        const bytes = await r.arrayBuffer();
        if (bytes.byteLength !== clip.bytes) throw Error('语音文件不完整');
        if (root.crypto?.subtle) {
          const hash = Array.from(new Uint8Array(await root.crypto.subtle.digest('SHA-256', bytes)))
            .map(n => n.toString(16).padStart(2, '0')).join('');
          if (hash !== clip.sha256) throw Error('语音文件校验失败');
        }
        return this.context.decodeAudioData(bytes);
      })();
      this.buffers.set(id, promise);
      try { return await promise; }
      catch (error) { this.buffers.delete(id); throw error; }
    }
    stopSource() {
      if (!this.source) return;
      const source = this.source;
      this.source = null;
      source.onended = null;
      try { source.stop(); } catch (_) { /* Already ended. */ }
      source.disconnect();
    }
    async enable() {
      this.cancelLegacySpeech();
      const Context = root.AudioContext || root.webkitAudioContext;
      if (!Context) { this.status('浏览器不支持音频，字幕继续'); return false; }
      try {
        if (!this.context) this.context = new Context();
        // Resume directly in the click handler to satisfy browser autoplay rules.
        const resumed = this.context.resume();
        this.lastKey = null;
        const token = ++this.generation;
        this.status('加载中文女声…');
        await resumed;
        await this.loadManifest();
        if (token !== this.generation) return false;
        this.claim();
        this.enabled = true;
        this.status('中文女声 · 离线可用');
        return true;
      } catch (_) {
        this.enabled = false;
        this.status('语音未就绪，请重试或检查语音包');
        return false;
      }
    }
    disable(message = '声音已关闭，字幕继续') {
      this.enabled = false;
      ++this.generation;
      this.stopSource();
      this.lastKey = null;
      this.cancelLegacySpeech();
      if (this.ownerStorage) {
        try { if (this.ownsAudio()) root.localStorage.removeItem(ownerKey); } catch (_) { /* Storage unavailable. */ }
      }
      this.status(message);
    }
    async say(id, key) {
      if (!this.enabled || key === this.lastKey) return false;
      if (!this.ownsAudio()) { this.disable('另一比赛页面已接管语音'); return false; }
      this.cancelLegacySpeech();
      this.lastKey = key;
      const token = ++this.generation;
      this.stopSource();
      this.status('加载中文女声…');
      try {
        const buffer = await this.buffer(id);
        // Ignore stale fetch/decode completions after a new phase or disable.
        if (!this.enabled || token !== this.generation) return false;
        if (!this.ownsAudio()) { this.disable('另一比赛页面已接管语音'); return false; }
        if (this.context.state !== 'running') {
          this.lastKey = null;
          this.status('音频已暂停，请关闭后重新开启');
          return false;
        }
        const source = this.context.createBufferSource();
        source.buffer = buffer;
        source.connect(this.context.destination);
        this.source = source;
        source.onended = () => {
          if (this.source !== source) return;
          this.source = null;
          source.disconnect();
          this.status('中文女声 · 离线可用');
        };
        source.start();
        this.status('正在播报 · 中文女声', true);
        return true;
      } catch (_) {
        if (this.enabled && token === this.generation) {
          this.lastKey = null;
          this.status('语音读取失败，请检查语音包');
        }
        return false;
      }
    }
  }
  root.CompetitionVoice = CompetitionVoice;
})(window);
