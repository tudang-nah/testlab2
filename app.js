'use strict';
const $ = s => document.querySelector(s), clamp = (v, a, b) => Math.min(b, Math.max(a, v));

// ---------- Parameters: [name,min,max,decimals,unit] ----------
const M = {
  P030:['Language',1,1,0,''],P031:['Motor NP Volts',20,460,0,'V'],P032:['Motor NP Hertz',15,500,0,'Hz'],
  P033:['Motor OL Current',0,99,1,'A'],P034:['Motor NP FLA',0,99,1,'A'],P035:['Motor NP Poles',2,40,0,''],
  P036:['Motor NP RPM',0,24000,0,'rpm'],P037:['Motor NP Power',0,99,2,'kW'],P039:['Torque Perf Mode',0,3,0,''],
  P040:['Autotune',0,2,0,''],P041:['Accel Time 1',0,600,2,'s'],P042:['Decel Time 1',0,600,2,'s'],
  P043:['Minimum Freq',0,500,2,'Hz'],P044:['Maximum Freq',0,500,2,'Hz'],P045:['Stop Mode',0,10,0,''],
  P046:['Start Source 1',1,5,0,''],P047:['Speed Reference1',1,16,0,''],P048:['Start Source 2',1,5,0,''],
  P049:['Speed Reference2',1,16,0,''],P050:['Start Source 3',1,5,0,''],P051:['Speed Reference3',1,16,0,''],
  P052:['Average kWh Cost',0,99,2,''],P053:['Reset To Defalts',0,3,0,''],
  t062:['DigIn TermBlk 02',0,70,0,''],t063:['DigIn TermBlk 03',0,70,0,''],t065:['DigIn TermBlk 05',0,70,0,''],t066:['DigIn TermBlk 06',0,70,0,''],
  A410:['Preset Freq 0',0,500,2,'Hz'],A411:['Preset Freq 1',0,500,2,'Hz'],A412:['Preset Freq 2',0,500,2,'Hz'],A413:['Preset Freq 3',0,500,2,'Hz'],
  A458:['PID Trim Sel',0,2,0,''],A459:['PID Ref Sel',0,13,0,''],A460:['PID Fdback Sel',0,6,0,''],
  A461:['PID Prop Gain',0,99.99,2,''],A462:['PID Integ Time',0,999.9,1,'s'],A463:['PID Diff Rate',0,99.99,2,'1/s'],
  A464:['PID Setpoint',0,100,1,'%'],A465:['PID Deadband',0,10,1,'%'],A466:['PID Preload',0,500,1,'Hz'],A544:['Reverse Disable',0,1,0,'']
};
const B = {b001:['Output Freq',1,'Hz'],b002:['Commanded Freq',1,'Hz'],b003:['Output Current',2,'A'],b004:['Output Voltage',0,'V'],
  b005:['DC Bus Voltage',0,'V'],b006:['Drive Status',0,''],b010:['Speed (rpm, mô phỏng)',0,'rpm']};
const DEF = {P030:1,P031:230,P032:60,P033:4.8,P034:2.9,P035:4,P036:1750,P037:.75,P039:1,P040:0,P041:10,P042:10,P043:0,P044:60,P045:0,
  P046:1,P047:1,P048:2,P049:5,P050:5,P051:15,P052:0,P053:0,t062:48,t063:50,t065:0,t066:0,A410:0,A411:5,A412:10,A413:20,
  A458:0,A459:0,A460:0,A461:.01,A462:2,A463:0,A464:0,A465:0,A466:0,A544:0};
const GL = {b:Object.keys(B), P:[], t:[], A:[]};
Object.keys(M).forEach(k => GL[k[0]].push(k));
const P = {...DEF};

// ---------- State ----------
const S = {f:0, tgt:0, rpm:0, ang:0, I:0, V:0, run:false, dir:0, curDir:0, latch:false, p2:false, fault:null, setF:0,
  sw:{'01':0,'02':0,'03':0,'05':0,'06':0}, integ:0, prevE:0, ol:0, hist:[]};
const ui = {m:'disp', g:'b', id:'b001', d:0, val:0};

// ---------- Helpers ----------
const msg = t => $('#msg').textContent = t;
const pot = () => +$('#pot').value, vr1 = () => +$('#vr1').value;
function meta(id){ return id==='FSET' ? ['Keypad Freq',S.setF*0+P.P043,P.P044,1,'Hz'] : M[id]; }
function getV(id){ return id==='FSET' ? S.setF : id[0]==='b' ? readB(id) : P[id]; }
function readB(id){
  return {b001:S.f, b002:S.tgt, b003:S.I, b004:S.V, b005:S.V>0||S.f>0?311:310, b006:S.fault?3:(S.f>0.01?1:0), b010:Math.abs(S.rpm)}[id];
}
function fmt(v,d){ return Number(v).toFixed(d); }

// ---------- Reference / PID ----------
function preset(){ return (P.t065==7&&S.sw['05']?1:0) + (P.t066==7&&S.sw['06']?2:0); }
function refHz(){
  switch(P.P047){
    case 1: return pot()/100*P.P044;
    case 2: return S.setF;
    case 5: return vr1()/10*P.P044;
    case 7: return P['A41'+preset()];
    case 11: return S.pid || 0;
    default: return 0;
  }
}
const refPct = () => ({0:P.A464,1:pot(),2:S.setF/Math.max(P.P044,1)*100,5:vr1()*10}[P.A459] ?? 0);
const fbPct = () => P.A460===0 ? vr1()*10 : 0;

// ---------- Simulation step ----------
function step(dt){
  const sw = S.sw; let run = false, dir = S.dir;
  if (P.P046===2){
    const fw = P.t062==48&&sw['02'], rv = P.t063==50&&sw['03'];
    if (fw&&rv){ S.latch=false; run=false; }
    else if (fw||rv){ run=true; dir = fw?0:1; }
    if (P.t062==49 && sw['02'] && !S.p2 && sw['01']) S.latch = true;
    if (S.latch){ run=true; dir = (P.t063==51&&sw['03'])?1:0; }
  } else { run = S.run; S.latch = false; }
  S.p2 = !!sw['02'];
  if (!sw['01']){ run=false; S.latch=false; S.run=false; }
  if (S.fault) run = false;
  S.cmdRun = run;

  // PID
  const pidOn = P.P047===11 && run;
  if (!pidOn){ S.integ = P.A466; S.prevE = 0; S.pid = P.A466; }
  else {
    let e = refPct() - fbPct(); if (Math.abs(e) <= P.A465) e = 0;
    const k = P.P044/100;
    if (P.A462 > 0) S.integ = clamp(S.integ + P.A461/P.A462*e*k*dt, 0, P.P044);
    const d = P.A461*P.A463*(e-S.prevE)/dt*k; S.prevE = e;
    S.pid = clamp(S.integ + P.A461*e*k + d, 0, P.P044);
  }

  // Frequency ramp
  let tgt = run ? clamp(refHz(), 0, P.P044) : 0;
  if (run && tgt>0 && tgt<P.P043) tgt = P.P043;
  if (run && dir!==S.curDir && S.f>0.01) tgt = 0;
  if (S.f<=0.01 && run) S.curDir = dir;
  S.dir = (P.P046===2) ? dir : S.dir;
  S.tgt = run ? clamp(refHz(),0,P.P044) : 0;
  const acc = P.P044/Math.max(P.P041,.01), dec = P.P044/Math.max(P.P042,.01);
  const prev = S.f;
  S.f = clamp(S.f + clamp(tgt-S.f, -dec*dt, acc*dt), 0, 500);
  const ramping = Math.abs(tgt-S.f) > 0.05;

  // Motor (V/Hz)
  S.V = S.f>0 ? Math.min(P.P031, P.P031*S.f/P.P032) : 0;
  const ns0 = 120*P.P032/P.P035, sRated = Math.max(0,(ns0-P.P036)/ns0), ns = 120*S.f/P.P035;
  const rpmT = (S.curDir?-1:1)*ns*(1-sRated*0.4);
  S.rpm += (rpmT-S.rpm)*(1-Math.exp(-dt/0.15));
  S.ang += S.rpm*0.25*dt;
  let I = S.f>0.01 ? P.P034*(0.45+0.2*Math.min(S.f/P.P032,1.5)) : 0;
  if (ramping && S.f>0.01) I += 0.4*P.P034;
  S.I = I ? I*(1+(Math.random()-.5)*0.02) : 0;
  S.ol = S.I>P.P033 ? S.ol+dt : Math.max(0,S.ol-dt);
  if (S.ol>60){ S.fault='F007'; S.ol=0; }
}

// ---------- Keypad / LCD ----------
function key(k){
  const id = ui.id;
  if (k==='Start'){
    if (P.P046!==1) return msg('Phím Start bị vô hiệu: P046 ≠ 1 (nguồn Start không phải Keypad).');
    if (!S.sw['01']) return msg('Công tắc 01 đang OFF (STOP) → động cơ không chạy được.');
    if (S.fault) return msg('Đang có lỗi: nhấn Stop để xóa lỗi.');
    S.run = true; return msg('Start từ keypad.');
  }
  if (k==='Stop'){ S.run=false; S.latch=false; if (S.fault){ S.fault=null; msg('Đã xóa lỗi.'); } return; }
  if (k==='Reverse'){
    if (P.P046!==1 || P.A544) return msg('Phím Reverse bị vô hiệu (P046 ≠ 1 hoặc A544 = 1).');
    S.dir ^= 1; return;
  }
  const m = ui.m;
  if (m==='disp'){
    if (k==='Esc'){ ui.m='num'; ui.id='b001'; }
    else if (k==='Up' && P.P047===2 && P.P046!==undefined){ ui.m='edit'; ui.id='FSET'; ui.d=0; ui.val=S.setF; }
    else if (k==='Up' || k==='Down') msg('Đặt tần số bằng phím chỉ có tác dụng khi P047 = 2 (Keypad Freq).');
  } else if (m==='grp'){
    const gs = Object.keys(GL), i = gs.indexOf(ui.g);
    if (k==='Up') ui.g = gs[(i+1)%gs.length]; if (k==='Down') ui.g = gs[(i+gs.length-1)%gs.length];
    if (k==='Enter'||k==='Sel'){ ui.m='num'; ui.id = GL[ui.g][0]; }
    if (k==='Esc') ui.m='disp';
    if (k==='Up'||k==='Down') ui.id = GL[ui.g][0];
  } else if (m==='num'){
    const L = GL[ui.g], i = L.indexOf(ui.id);
    if (k==='Up') ui.id = L[(i+1)%L.length]; if (k==='Down') ui.id = L[(i+L.length-1)%L.length];
    if (k==='Enter'||k==='Sel') ui.m='val';
    if (k==='Esc') ui.m='grp';
  } else if (m==='val'){
    if (k==='Esc') ui.m='num';
    if (k==='Enter'||k==='Sel'){
      if (id[0]==='b') return msg('Thanh ghi chỉ đọc.');
      if (S.run && S.cmdRun && (id[0]==='P'||id[0]==='t')) return msg('Dừng động cơ trước khi đổi thanh ghi này.');
      ui.m='edit'; ui.d=0; ui.val=getV(id);
    }
  } else if (m==='edit'){
    const [,mn,mx,dec] = meta(id);
    if (k==='Up'||k==='Down') ui.val = clamp(+(ui.val + (k==='Up'?1:-1)*Math.pow(10,ui.d-dec)).toFixed(dec), mn, mx);
    if (k==='Sel') ui.d = (ui.d+1) % (dec + (mx>=100?3:2));
    if (k==='Esc'){ ui.m = id==='FSET' ? 'disp' : 'val'; }
    if (k==='Enter'){
      if (id==='FSET') S.setF = clamp(ui.val, 0, P.P044);
      else { P[id] = ui.val; if (id==='P053'&&P.P053>0) factory(); if (id==='P044') S.setF = Math.min(S.setF,P.P044); }
      ui.m = id==='FSET' ? 'disp' : 'val';
    }
  }
}
function render(){
  const id = ui.id; let main='', unit='', name='';
  if (ui.m==='disp'){ main = S.fault ? `<span class="blink">${S.fault}</span>` : fmt(S.f,1); unit='HERTZ'; }
  else if (ui.m==='grp'){ main = `<span class="blink">${ui.g}</span>${id.slice(1)}`; }
  else if (ui.m==='num'){ main = `${id[0]}<span class="blink">${id.slice(1)}</span>`; name = (M[id]||B[id])[0]; }
  else {
    const mt = id==='FSET' ? meta(id) : (M[id]||[B[id][0],0,0,B[id][1],B[id][2]]);
    const dec = mt[3]; unit = (mt[4]||'').toUpperCase(); name = id+' '+mt[0];
    if (ui.m==='val') main = fmt(getV(id),dec);
    else {
      const s = fmt(ui.val,dec).split(''); let k = s.length-1, c = 0;
      while (k>=0){ if (s[k]!=='.'){ if (c===ui.d) break; c++; } k--; }
      if (k>=0) s[k] = `<span class="blink">${s[k]}</span>`;
      main = s.join('');
    }
  }
  $('#lMain').innerHTML = main; $('#lUnit').textContent = unit; $('#lName').textContent = name;
  $('#lPrg').classList.toggle('on', ui.m==='edit' && ui.id!=='FSET');
  $('#lFwd').classList.toggle('on', S.f>0.01 ? !S.curDir : !S.dir);
  $('#lRev').classList.toggle('on', S.f>0.01 ? !!S.curDir : !!S.dir);
  $('#lFault').classList.toggle('on', !!S.fault);
  $('#rF').textContent = fmt(S.f,1)+' Hz'; $('#rN').textContent = Math.round(Math.abs(S.rpm))+' rpm';
  $('#rV').textContent = Math.round(S.V)+' V'; $('#rI').textContent = fmt(S.I,2)+' A';
  $('#rD').textContent = S.f>0.01 ? (S.curDir?'Nghịch':'Thuận') : '–';
  $('#rotor').setAttribute('transform', `rotate(${S.ang%360})`);
  $('#potV').textContent = pot()+'%'; $('#vr1V').textContent = vr1().toFixed(2)+' V';
  const run = S.f>0.01; ['R1','R2'].forEach(r=>$('#'+r).classList.toggle('on',!!S.fault)); ['R5','R6'].forEach(r=>$('#'+r).classList.toggle('on',run));
}

// ---------- Factory reset / presets ----------
function factory(){
  Object.assign(P, DEF); S.setF = 0; S.run = false; S.latch = false; S.fault = 'F048'; S.f = 0; S.pid = 0;
  msg('Factory Reset xong: lỗi F048 (Params Default). Nhấn Stop để xóa lỗi rồi cấu hình lại P030–P052.');
}
const MOTOR = {P030:1,P031:220,P032:50,P033:1.8,P034:1.2,P035:4,P036:1350,P037:.18,P039:0,P040:0,P041:7.5,P042:7.5,P043:0,P044:75,P045:0};
const T = {
  'TN1 Thông số động cơ':[{P046:1,P047:1},{},'Đã nạp a1–a7, b1, c1–c3 (220 V, 50 Hz, 1,8 A, 1,2 A, 4 cực, 1350 rpm, 0,18 kW, V/Hz, 7,5 s, 75 Hz). Hãy thử tự nhập bằng keypad trước!'],
  'TN2.1 Keypad + phím':[{P046:1,P047:2},{'01':1},'Nhấn ▲ ở màn hình chính, đặt 25 Hz, Enter; Start; Reverse. Thử 70 Hz và 120 Hz: 120 Hz bị chặn ở 75 Hz (P044).'],
  'TN2.2 Keypad + DrivePot':[{P046:1,P047:1},{'01':1},'Start rồi xoay DrivePot: tần số 0 → 75 Hz. Tốc độ xem ở b010.'],
  'TN2.3 DI 2-wire':[{P046:2,P047:2,t062:48,t063:50},{'01':1},'Đặt 25 Hz bằng ▲. Bật 02 → thuận; tắt 02 rồi bật 03 → nghịch; bật cả 02 và 03 → dừng.',25],
  'TN2.4 DI 3-wire':[{P046:2,P047:2,t062:49,t063:51},{'01':1},'Bật 02 (xung Start) → chạy; tắt 02 vẫn chạy (tự giữ); 03 = chiều; Stop/01 OFF để dừng. Phím Start keypad bị vô hiệu.',25],
  'TN2.5 Preset Freq':[{P046:1,P047:7,t065:7,t066:7,A410:0,A411:15,A412:30,A413:45},{'01':1},'Nhấn Start; đổi công tắc 05/06 và ghi tần số (05 = bit nhỏ). Thử thứ tự Gray: A412 = 45, A413 = 30.'],
  'TN2.6 Biến trở Vr1':[{P046:1,P047:5},{'01':1},'Start rồi xoay Vr1 (0–10 V → 0–75 Hz). Ghi tốc độ, điện áp, dòng ở 10/25/45 Hz.'],
  'TN3 PID bơm':[{P046:1,P047:11,A458:0,A459:1,A460:0,A461:.05,A462:2,A463:1,A465:0,A466:0},{'01':1},'DrivePot = giá trị đặt, Vr1 = hồi tiếp. Start; hồi tiếp thấp → tần số tăng, cao → giảm, bằng → xác lập. Đặt A465 = 5 để thử Deadband.']
};
function apply(name){
  const [cfg, sws, hint, f] = T[name];
  Object.assign(P, DEF, MOTOR, cfg); S.setF = f||0; S.run=false; S.latch=false; S.fault=null; S.f=0; S.rpm=0;
  Object.keys(S.sw).forEach(k => S.sw[k] = sws[k]||0); buildSw();
  $('#phint').textContent = hint; msg('Đã nạp '+name+'. Công tắc 01 '+(S.sw['01']?'đã ON.':'đang OFF.'));
  document.querySelectorAll('#presets button').forEach(b => b.classList.toggle('act', b.textContent===name));
}

// ---------- UI wiring ----------
function buildSw(){
  const box = $('#sws'); box.innerHTML = '';
  Object.keys(S.sw).forEach(k => {
    const b = document.createElement('button'); b.className = S.sw[k]?'on':''; b.innerHTML = `<i class="sk"></i><span>${k}</span><small>${S.sw[k]?'ON 24V':'OFF 0V'}</small>`;
    b.onclick = () => { S.sw[k] ^= 1; buildSw(); }; box.appendChild(b);
  });
}
document.querySelectorAll('[data-k]').forEach(b => b.onclick = () => key(b.dataset.k));
Object.keys(T).forEach(n => { const b = document.createElement('button'); b.textContent = n; b.onclick = () => apply(n); $('#presets').appendChild(b); });
$('#factory').onclick = () => { P.P053 = 2; factory(); P.P053 = 0; };
$('#add').onclick = () => {
  const r = [fmt(S.f,1), Math.round(Math.abs(S.rpm)), Math.round(S.V), fmt(S.I,2)], tr = document.createElement('tr');
  tr.innerHTML = r.map(x=>`<td>${x}</td>`).join('') + '<td contenteditable></td>'; $('#tb tbody').appendChild(tr);
};
$('#clr').onclick = () => $('#tb tbody').innerHTML = '';
$('#copy').onclick = () => {
  const rows = [['Hz','rpm','V','A','ghi chu'], ...[...document.querySelectorAll('#tb tbody tr')].map(tr => [...tr.children].map(td=>td.textContent))];
  navigator.clipboard.writeText(rows.map(r=>r.join(',')).join('\n')).then(()=>msg('Đã sao chép CSV.'));
};
buildSw();

// ---------- Chart ----------
function draw(){
  const c = $('#cv'), g = c.getContext('2d'), w = c.width, h = c.height, mx = Math.max(P.P044,1);
  g.clearRect(0,0,w,h); g.strokeStyle = '#8886'; g.strokeRect(0,0,w,h);
  const line = (key, col, dash) => { g.beginPath(); g.strokeStyle = col; g.setLineDash(dash||[]); g.lineWidth = 2;
    S.hist.forEach((p,i) => { const x = i/600*w, y = h-6-clamp(p[key],0,1)*(h-12); i ? g.lineTo(x,y) : g.moveTo(x,y); }); g.stroke(); };
  line(0,'#2e9b4a'); if (P.P047===11){ line(1,'#2f6fd0',[5,4]); line(2,'#e07a1f'); }
}

// ---------- Main loop ----------
let tick = 0;
setInterval(() => {
  const n = +$('#speed').value; for (let i=0;i<n;i++) step(0.05);
  if (++tick % 2===0){ S.hist.push([S.f/Math.max(P.P044,1), refPct()/100, fbPct()/100]); if (S.hist.length>600) S.hist.shift(); }
  render(); draw();
}, 50);

// ---------- Rotary knobs ----------
document.querySelectorAll('.rk').forEach(k => {
  const inp = $('#'+k.dataset.for), mn = +inp.min, mx = +inp.max;
  const set = v => { inp.value = v; k.firstElementChild.style.transform = `rotate(${-135+(v-mn)/(mx-mn)*270}deg)`; };
  const mv = e => { const r = k.getBoundingClientRect(); let a = Math.atan2(e.clientX-r.left-r.width/2, -(e.clientY-r.top-r.height/2))*180/Math.PI;
    set(+(mn+(clamp(a,-135,135)+135)/270*(mx-mn)).toFixed(2)); };
  k.onpointerdown = e => { k.setPointerCapture(e.pointerId); mv(e); k.onpointermove = mv; };
  k.onpointerup = () => k.onpointermove = null;
  k.onwheel = e => { e.preventDefault(); set(clamp(+inp.value+(e.deltaY<0?1:-1)*(mx-mn)/50, mn, mx)); };
  set(+inp.value);
});
