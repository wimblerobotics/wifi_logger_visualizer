#!/usr/bin/env python3
import argparse
import errno
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import html
import io
import json
from pathlib import Path
import sqlite3
import socket
from threading import Lock
from time import monotonic
from urllib.parse import parse_qs, urlparse

import matplotlib.pyplot as plt

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.map_data import decode_pgm, free_cell_centers
from wifi_logger_visualizer.orientation_data import load_saved_orientation
from wifi_logger_visualizer.map_export import export_map_bundle
import yaml


HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>__ROBOT_NAME__ Wi-Fi Map</title>
<style>
* { box-sizing:border-box; }
html,body { width:100%; height:100%; }
body { font-family:system-ui,sans-serif; margin:0; padding:1rem;
 background:#202124; color:#eee; display:flex; flex-direction:column; gap:.75rem; }
h2 { margin:0; font-size:1.2rem; font-weight:600; }
#toolbar { display:flex; gap:.75rem; align-items:center; flex-wrap:wrap; flex-shrink:0; }
#toolbar label { display:flex; align-items:center; gap:.4rem; min-width:0; }
#mapSelect { max-width:calc(100vw - 6rem); }
#content { position:relative; flex:1; min-height:0; min-width:0;
 background:#151618; border:1px solid #444; border-radius:.6rem; overflow:hidden; }
#viewport { position:absolute; inset:0; display:flex; align-items:center; justify-content:center;
 overflow:hidden; cursor:grab; touch-action:none; user-select:none; }
#viewport.dragging { cursor:grabbing; }
#wrap { position:relative; flex:none; }
#map { pointer-events:none; display:block; width:100%; height:100%; image-rendering:pixelated; }
#overlay { position:absolute; left:0; top:0; width:100%; height:100%; }
#legend { position:absolute; right:.75rem; top:.75rem; max-width:min(20rem,45%);
 max-height:calc(100% - 1.5rem); overflow:auto; padding:.75rem;
 background:rgba(32,33,36,.9); border:1px solid #555; border-radius:.5rem;
 box-shadow:0 2px 10px #0005; font-size:.85rem; overflow-wrap:anywhere; }
#legendTitle { font-weight:bold; margin-bottom:.75rem; }
#compass { position:absolute; left:.75rem; top:.75rem; max-width:min(15rem,45%);
 padding:.5rem .65rem; background:rgba(32,33,36,.9); border:1px solid #555;
 border-radius:.5rem; font-size:.75rem; overflow-wrap:anywhere; }
#compassRose { display:none; width:100px; height:100px; margin:auto; }
#compassTitle { font-weight:600; text-align:center; }
#compassDetails { margin-top:.3rem; }
.legendScale { display:flex; gap:.5rem; height:15rem; }
.legendBar { width:1.5rem; border:1px solid #aaa; }
.legendLabels { display:flex; flex-direction:column; justify-content:space-between; }
.legendEntry { display:flex; gap:.5rem; align-items:center; margin:.4rem 0; }
.legendSwatch { width:1.25rem; height:1.25rem; border:1px solid #aaa; }
#tip { min-height:9.5em; max-height:32.0vh; overflow:auto; padding:.65rem .8rem;
 font-size:.85rem; line-height:1.5; flex-shrink:0; white-space:pre-line;
 background:#292a2d; border:1px solid #444; border-radius:.5rem; }
#loadStatus:empty { display:none; }
select,button { padding:.4rem .55rem; border:1px solid #666; border-radius:.3rem;
 background:#303134; color:#eee; }
button { cursor:pointer; }
button:hover { background:#45464a; }
@media (max-width:600px) {
 body { padding:.5rem; gap:.5rem; }
 #legend { padding:.5rem; font-size:.75rem; }
 .legendScale { height:8rem; }
}
</style></head>
<body><h2>__ROBOT_NAME__ Wi-Fi coverage</h2><div id="toolbar">
<label>Map <select id="mapSelect"></select></label>
<label>Layer <select id="metric">
<option value="signal">Signal (dBm)</option><option value="tx">TX link rate</option>
<option value="rx">RX link rate</option><option value="upload">Upload to server</option>
<option value="download">Download from server</option>
<option value="idle_latency">Idle latency</option>
<option value="loaded_latency">Loaded latency</option>
<option value="age">Observation age</option><option value="bssid">Access point</option>
</select></label>
<label><input id="unmeasured" type="checkbox"> Highlight unmeasured free cells</label>
<label>Map name <input id="mapName" type="text" maxlength="100" placeholder="office" value="map"></label><button id="saveMap">Save map files</button>
<button id="keepRun" disabled hidden>Promote</button><button id="fitMap" title="Reset zoom and pan">Fit map</button><button id="refresh">Refresh</button><button id="export">Export PNG</button></div>
<div id="content"><div id="viewport" title="Scroll to zoom; drag to pan; double-click to fit"><div id="wrap"><img id="map" alt="Wi-Fi coverage map"><canvas id="overlay"></canvas></div></div>
<div id="compass"><div id="compassTitle">Saved orientation unavailable</div>
<canvas id="compassRose" width="120" height="120" role="img" aria-label="Saved true North direction"></canvas>
<div id="compassDetails" role="status"></div></div>
<div id="legend"><div id="legendTitle"></div><div id="legendBody"></div></div></div>
<div id="loadStatus" role="status"></div><div id="tip">Hover over a measured cell to see its Wi-Fi details.</div>
<script>
const ranges={signal:[-90,-35],tx:[0,1200],rx:[0,1200],upload:[0,500],
 download:[0,500],idle_latency:[0,100],loaded_latency:[0,200],age:[0,30]};
const titles={signal:'Signal strength',tx:'TX link rate',rx:'RX link rate',
 upload:'Upload to server',download:'Download from server',
 idle_latency:'Idle latency',loaded_latency:'Loaded latency',
 age:'Observation age',bssid:'Access point'};
const units={signal:'dBm',tx:'Mb/s',rx:'Mb/s',upload:'Mb/s',download:'Mb/s',
 idle_latency:'ms',loaded_latency:'ms',age:'days'};
const unmeasuredColor='rgba(65,105,225,.48)';
const nodeNames=__NODE_NAMES__;
const useGps=__USE_GPS__;
const readOnly=__READ_ONLY__;
function apLabel(bssid){
 if(!bssid)return 'Unknown Node';
 return (nodeNames[bssid.toLowerCase()]||'Unknown Node')+' ('+bssid+')';
}
function gpsFixStatus(status){
 if(!Number.isInteger(status))return 'Not available';
 return ({'-2':'Unknown','-1':'No fix','0':'Fix (unaugmented)',
  '1':'Fix with satellite-based augmentation (SBAS)',
  '2':'Fix with ground-based augmentation (GBAS)'})[status]??`Unknown (${status})`;
}
function gpsService(service){
 if(!Number.isInteger(service)||service<0)return 'Not available';
 if(service===0)return 'Unknown';
 const systems=[[1,'GPS'],[2,'GLONASS'],[4,'BeiDou (COMPASS)'],[8,'Galileo']];
 const names=systems.filter(([bit])=>(service&bit)!==0).map(([,name])=>name);
 const unknown=service&~15;
 if(unknown)names.push(`Unknown flags (${unknown})`);
 return names.join(' + ');
}
function gpsDetails(cell){
 const value=v=>v===null||v===undefined||v===''?'Not available':String(v);
 const coordinate=v=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(7)+'°':'Not available';
 const altitude=typeof cell.altitude==='number'&&Number.isFinite(cell.altitude)?
  cell.altitude.toFixed(3)+' m (WGS 84)':'Not available';
 return ` ·  ·  · GPS: lat: ${coordinate(cell.latitude)}, long: ${coordinate(cell.longitude)}, alt: ${altitude}`
  + ` ·  ·  · Fix status: ${gpsFixStatus(cell.gps_status)} · Service: ${gpsService(cell.gps_service)}`
  + ` · GPS stamp (ROS ns): ${value(cell.gps_stamp_ns)} · Frame: ${value(cell.gps_frame_id)}`
  //+ ` · Covariance (ENU, m²): ${value(cell.gps_covariance)} · Covariance type: ${value(cell.gps_covariance_type)}`;
}
let payload=null, drawn=[];
function color(value, metric) {
 if(metric==='bssid'){let h=0; for(const c of (value||'')) h=(h*31+c.charCodeAt(0))%360;
   return `hsla(${h},85%,50%,.68)`;}
 let [lo,hi]=ranges[metric], n=Math.max(0,Math.min(1,(value-lo)/(hi-lo)));
 if(metric.includes('latency')||metric==='age') n=1-n;
 return `hsla(${n*120},100%,50%,.68)`;
}
function valueFor(cell,metric){
 const survey=payload.surveys.find(s=>s.cell_x===cell.cell_x&&s.cell_y===cell.cell_y);
 if(metric==='signal')return cell.signal_dbm_last;
 if(metric==='tx')return cell.tx_bitrate_mbps_last;
 if(metric==='rx')return cell.rx_bitrate_mbps_last;
 if(metric==='bssid')return cell.bssid;
 if(metric==='age')return (Date.now()-Date.parse(cell.last_observed_at))/86400000;
 if(!survey)return null;
 return survey[metric+'_ms'] ?? survey[metric+'_mbps'];
}
function pixel(x,y){const m=payload.map; return [(x-m.origin_x)/m.resolution,
 m.height-(y-m.origin_y)/m.resolution];}
function drawCompass(context,x,y,size,rotation){
 context.save();context.translate(x+size/2,y+size/2);context.scale(size/120,size/120);
 context.fillStyle='rgba(32,33,36,.9)';context.fillRect(-60,-60,120,120);
 context.strokeStyle='#aaa';context.lineWidth=1;context.beginPath();
 context.arc(0,0,34,0,Math.PI*2);context.stroke();
 const angle=rotation*Math.PI/180;
 context.save();context.rotate(angle);context.strokeStyle='#ff7777';context.lineWidth=3;
 context.beginPath();context.moveTo(0,22);context.lineTo(0,-26);context.stroke();
 context.fillStyle='#ff7777';context.beginPath();context.moveTo(0,-33);
 context.lineTo(-7,-19);context.lineTo(7,-19);context.closePath();context.fill();context.restore();
 context.font='bold 14px system-ui,sans-serif';context.textAlign='center';context.textBaseline='middle';
 for(const [label,offset] of [['N',-Math.PI/2],['E',0],['S',Math.PI/2],['W',Math.PI]]){
  context.fillStyle=label==='N'?'#ff9999':'#eee';
  context.fillText(label,Math.cos(angle+offset)*47,Math.sin(angle+offset)*47);
 }
 context.restore();
}
function drawOrientation(){
 const orientation=payload.orientation;
 const rose=document.getElementById('compassRose'),title=document.getElementById('compassTitle');
 const details=document.getElementById('compassDetails');
 const available=orientation?.valid===true;
 rose.style.display=available?'block':'none';
 const live=payload.run?.map_source==='live';
 title.textContent=available?(live?'True North (captured)':'True North (saved)'):'Orientation unavailable';
 document.getElementById('compass').title=available&&orientation.saved_at?
  (live?'Orientation captured: ':'Map saved: ')+orientation.saved_at:'';
 if(!available){details.textContent=orientation?.reason||'No saved orientation';return;}
 drawCompass(rose.getContext('2d'),0,0,120,orientation.north_rotation_in_image_deg);
 const uncertainty=typeof orientation.uncertainty_deg==='number'?
  ` · ±${orientation.uncertainty_deg.toFixed(1)}°`:' · Uncertainty unavailable';
 const heading=orientation.map_x_heading_from_east_deg;
 details.textContent=`Map +X: ${(Math.abs(heading)<.05?0:heading).toFixed(1)}° from East${uncertainty}`;
}
function drawLegend(metric){
 const title=document.getElementById('legendTitle');
 const body=document.getElementById('legendBody');title.textContent=titles[metric];
 body.replaceChildren();
 if(metric==='bssid'){
  const values=[...new Set(payload.cells.map(c=>c.bssid).filter(Boolean))]
   .sort((a,b)=>apLabel(a).localeCompare(apLabel(b)));
  for(const value of values){const row=document.createElement('div');
   row.className='legendEntry';const swatch=document.createElement('span');
   swatch.className='legendSwatch';swatch.style.background=color(value,metric);
   const label=document.createElement('span');label.textContent=apLabel(value);
   row.append(swatch,label);body.append(row);}
  if(!values.length)body.textContent='No access points measured';
  return;
 }
 const [low,high]=ranges[metric],reverse=metric.includes('latency')||metric==='age';
 const scale=document.createElement('div');scale.className='legendScale';
 const bar=document.createElement('div');bar.className='legendBar';
 bar.style.background=reverse?'linear-gradient(to top,#00ff00,#ff0000)':
  'linear-gradient(to top,#ff0000,#00ff00)';
 const labels=document.createElement('div');labels.className='legendLabels';
 const highLabel=document.createElement('span'),lowLabel=document.createElement('span');
 highLabel.textContent=`${high} ${units[metric]}`;
 lowLabel.textContent=`${low} ${units[metric]}`;
 labels.append(highLabel,lowLabel);scale.append(bar,labels);body.append(scale);
 if(document.getElementById('unmeasured').checked){
  const note=document.createElement('div');note.className='legendEntry';
  const swatch=document.createElement('span');swatch.className='legendSwatch';
  swatch.style.background=unmeasuredColor;
  const label=document.createElement('span');label.textContent='Unmeasured free space';
  note.append(swatch,label);body.append(note);}
}
let mapZoom=1, mapPanX=0, mapPanY=0, mapDrag=null, viewedMapId=null;
function resetMapView(){
 mapZoom=1;mapPanX=0;mapPanY=0;fitMap();
}
function fitMap(){
 const img=document.getElementById('map'),viewport=document.getElementById('viewport');
 if(!img.naturalWidth||!viewport.clientWidth||!viewport.clientHeight)return;
 const scale=Math.min(viewport.clientWidth/img.naturalWidth,viewport.clientHeight/img.naturalHeight);
 const wrap=document.getElementById('wrap');
 wrap.style.width=img.naturalWidth*scale*mapZoom+'px';
 wrap.style.height=img.naturalHeight*scale*mapZoom+'px';
 wrap.style.transform=`translate(${mapPanX}px,${mapPanY}px)`;
}
const viewport=document.getElementById('viewport');
new ResizeObserver(fitMap).observe(viewport);
viewport.addEventListener('wheel',e=>{
 if(!document.getElementById('map').naturalWidth)return;
 e.preventDefault();
 const rect=viewport.getBoundingClientRect();
 const x=e.clientX-rect.left-rect.width/2,y=e.clientY-rect.top-rect.height/2;
 const delta=e.deltaY*(e.deltaMode===1?16:e.deltaMode===2?viewport.clientHeight:1);
 const zoom=Math.max(1,Math.min(20,mapZoom*Math.exp(-delta*.0015)));
 const ratio=zoom/mapZoom;
 mapPanX=x-(x-mapPanX)*ratio;mapPanY=y-(y-mapPanY)*ratio;mapZoom=zoom;
 if(mapZoom===1){mapPanX=0;mapPanY=0;}
 fitMap();
},{passive:false});
viewport.addEventListener('pointerdown',e=>{
 if(e.button!==0)return;
 mapDrag={id:e.pointerId,x:e.clientX,y:e.clientY,panX:mapPanX,panY:mapPanY};
 viewport.setPointerCapture(e.pointerId);viewport.classList.add('dragging');
 e.preventDefault();
});
viewport.addEventListener('pointermove',e=>{
 if(!mapDrag||mapDrag.id!==e.pointerId)return;
 mapPanX=mapDrag.panX+e.clientX-mapDrag.x;mapPanY=mapDrag.panY+e.clientY-mapDrag.y;
 fitMap();
});
function endMapDrag(e){
 if(!mapDrag||mapDrag.id!==e.pointerId)return;
 mapDrag=null;viewport.classList.remove('dragging');
 if(viewport.hasPointerCapture(e.pointerId))viewport.releasePointerCapture(e.pointerId);
}
viewport.addEventListener('pointerup',endMapDrag);
viewport.addEventListener('pointercancel',endMapDrag);
viewport.addEventListener('lostpointercapture',endMapDrag);
viewport.addEventListener('dblclick',resetMapView);
document.getElementById('fitMap').onclick=resetMapView;
function draw(){
 if(!payload || !document.getElementById("map").naturalWidth)return;
 const img=document.getElementById('map'), c=document.getElementById('overlay');
 c.width=img.naturalWidth;c.height=img.naturalHeight;const x=c.getContext('2d');
 x.clearRect(0,0,c.width,c.height);drawn=[];
 const metric=document.getElementById('metric').value;
 drawOrientation();
 drawLegend(metric);
 const size=payload.cell_size/payload.map.resolution;
 const measured=new Set(payload.cells.map(v=>v.cell_x+','+v.cell_y));
 if(document.getElementById('unmeasured').checked){
  x.fillStyle=unmeasuredColor;
  for(const p of payload.free_cells){const [px,py]=pixel(p[0],p[1]);
   const key=Math.floor(p[0]/payload.cell_size)+','+Math.floor(p[1]/payload.cell_size);
   if(!measured.has(key))x.fillRect(px-size/2,py-size/2,size,size);}
 }
 for(const cell of payload.cells){const v=valueFor(cell,metric);
  if(v===null||v===undefined)continue;
  const [px,py]=pixel(cell.center_x,cell.center_y);x.fillStyle=color(v,metric);
  x.fillRect(px-size/2,py-size/2,size,size);drawn.push({px,py,size,cell,value:v});}
}
let loading=false;
async function loadData(){
 if(loading)return;
 loading=true;
 const status=document.getElementById('loadStatus');
 status.textContent='Loading map data...';
 try{
  const id=document.getElementById('mapSelect').value;
  const url='/api/data'+(id?'?map_id='+encodeURIComponent(id):'');
  let data;
  for(let attempt=0;attempt<2;attempt++){
   try{
    const response=await fetch(url,{cache:'no-store',signal:AbortSignal.timeout(15000)});
    if(!response.ok)throw new Error(`HTTP ${response.status}: ${await response.text()}`);
    data=await response.json();
    break;
   }catch(error){
    if(!['AbortError','TimeoutError'].includes(error.name)||attempt===1)throw error;
    status.textContent='Data request was interrupted or timed out; retrying...';
    await new Promise(resolve=>setTimeout(resolve,300));
   }
  }
  if(!data.map||!Array.isArray(data.maps))throw new Error('Invalid map data response');
  payload=data;
  const sel=document.getElementById('mapSelect');
  sel.replaceChildren();
  for(const m of payload.maps){const o=document.createElement('option');o.value=m.id;
   const saved=m.map_source!=='live'&&m.orientation_saved_at ? new Date(m.orientation_saved_at) : null;
   const hasSaved=saved&&!Number.isNaN(saved.getTime());
   const timestamp=hasSaved ? saved : (m.captured_at ? new Date(m.captured_at) : null);
   const timeLabel=timestamp&&!Number.isNaN(timestamp.getTime())
    ? ` — ${hasSaved ? 'saved' : 'captured'} ${timestamp.toLocaleString(undefined,{timeZoneName:'short'})}` : '';
   o.text=`${m.id}: ${m.source_yaml_path} (${m.content_hash.slice(0,8)})${timeLabel}${m.temporary ? ' [temporary]' : ''}`;sel.add(o);}
  sel.value=payload.map.id;
  const run=payload.run;
  const keep=document.getElementById('keepRun');
  keep.hidden=readOnly;
  keep.disabled=readOnly||!run||!run.temporary;
  keep.textContent=run?.temporary?'Promote':'Run retained';
  const img=document.getElementById('map');
  img.onload=()=>{
   if(viewedMapId!==payload.map.id){viewedMapId=payload.map.id;resetMapView();}
   else fitMap();
   draw();status.textContent='';
  };
  img.onerror=()=>{status.textContent='Map image failed to load. Click Refresh to retry.';};
  img.src='/map.png?map_id='+payload.map.id+'&t='+Date.now();
 }catch(error){
  console.error('Wi-Fi map load failed:',error);
  status.textContent=`Could not load map data: ${error.message}. Click Refresh to retry.`;
 }finally{loading=false;}
}
document.getElementById('metric').onchange=draw;
document.getElementById('unmeasured').onchange=draw;
document.getElementById('mapSelect').onchange=loadData;
document.getElementById('refresh').onclick=loadData;
document.getElementById('saveMap').onclick=async()=>{
 const button=document.getElementById('saveMap'),status=document.getElementById('loadStatus');
 if(!payload?.map){status.textContent='Wait for a map to load before saving.';return;}
 const name=document.getElementById('mapName').value.trim();
 button.disabled=true;status.textContent='Preparing map files...';
 try{
  const response=await fetch('/api/export-map',{method:'POST',headers:{'Content-Type':'application/json'},
   body:JSON.stringify({map_id:payload.map.id,name})});
  if(!response.ok)throw new Error(await response.text());
  const blob=await response.blob(),url=URL.createObjectURL(blob),link=document.createElement('a');
  link.href=url;link.download=name+'.zip';document.body.append(link);link.click();link.remove();
  setTimeout(()=>URL.revokeObjectURL(url),60000);
  status.textContent='Map files downloaded. Extract the ZIP before loading the map YAML.';
 }catch(error){status.textContent=`Could not save map: ${error.message}`;}
 finally{button.disabled=false;}
};
document.getElementById('keepRun').onclick=async()=>{
 const button=document.getElementById('keepRun');button.disabled=true;
 try{
  const response=await fetch('/api/keep-run',{method:'POST',headers:{'Content-Type':'application/json'},
   body:JSON.stringify({run_id:payload.run.id})});
  if(!response.ok)throw new Error(`HTTP ${response.status}`);
  await loadData();
 }catch(error){document.getElementById('loadStatus').textContent=`Could not keep run: ${error.message}`;
  button.disabled=false;}
};
document.getElementById('export').onclick=()=>{
 const image=document.getElementById('map'),overlay=document.getElementById('overlay');
 const output=document.createElement('canvas');output.width=overlay.width;
 output.height=overlay.height;const context=output.getContext('2d');
 context.drawImage(image,0,0);context.drawImage(overlay,0,0);
 if(payload?.orientation?.valid){
  const size=Math.min(120,output.width/3,output.height/3);
  drawCompass(context,4,4,size,payload.orientation.north_rotation_in_image_deg);
 }
 const link=document.createElement('a');link.download=__MAP_DOWNLOAD_NAME__;
 link.href=output.toDataURL('image/png');link.click();};
document.getElementById('overlay').onmousemove=e=>{
 if(mapDrag)return;
 const r=e.target.getBoundingClientRect();
 const px=(e.clientX-r.left)*e.target.width/r.width,py=(e.clientY-r.top)*e.target.height/r.height;
 const hit=drawn.find(v=>Math.abs(v.px-px)<v.size/2&&Math.abs(v.py-py)<v.size/2);
 const tip=document.getElementById('tip');
 if(!hit){tip.textContent='Hover over a measured cell to see its Wi-Fi details.';return;}
 const cell=hit.cell,metric=document.getElementById('metric').value;
 const number=(value,unit)=>typeof value==='number'&&Number.isFinite(value)?
  `${value.toFixed(1)} ${unit}`:'Not available';
 const selected=metric==='bssid'?apLabel(hit.value):number(hit.value,units[metric]);
 const observed=new Date(cell.last_observed_at);
 const observedText=Number.isNaN(observed.getTime())?'Not available':
  observed.toLocaleString(undefined,{timeZoneName:'short'});
 tip.textContent=[
  `Position: X ${cell.center_x.toFixed(2)} m, Y ${cell.center_y.toFixed(2)} m · Cell (${cell.cell_x}, ${cell.cell_y})${useGps?' · '+gpsDetails(cell):''}`,
  `Access point: ${apLabel(cell.bssid)}`,
  `${titles[metric]}: ${selected}`,
  `Signal: latest ${number(cell.signal_dbm_last,'dBm')} · weakest ${number(cell.signal_dbm_min,'dBm')} · strongest ${number(cell.signal_dbm_max,'dBm')}`,
  `Last observed: ${observedText} [${cell.last_observed_at ?? 'Not available'}]`
 ].join('\\n');};
loadData();
</script></body></html>"""


def load_node_names(path: str) -> dict[str, str]:
    config = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(config, dict) or not isinstance(
        config.get("access_points"), dict
    ):
        raise ValueError(
            f"Access point configuration must contain an access_points mapping: {path}"
        )
    return {
        str(bssid).lower(): str(name)
        for bssid, name in config["access_points"].items()
    }


def default_node_names_path() -> Path:
    from ament_index_python.packages import get_package_share_directory

    return (
        Path(get_package_share_directory("wifi_logger_visualizer"))
        / "config"
        / "access_points.yaml"
    )


def robot_name() -> str:
    """Use the local machine name without its network domain suffix."""
    return socket.gethostname().split(".", 1)[0].capitalize()


def rendered_html(node_names: dict[str, str], use_gps: bool = False, read_only: bool = True) -> str:
    name = robot_name()
    return (HTML.replace("__ROBOT_NAME__", html.escape(name))
            .replace("__MAP_DOWNLOAD_NAME__", json.dumps(name.lower() + "-wifi-map.png"))
            .replace("__NODE_NAMES__", json.dumps(node_names, separators=(",", ":")))
            .replace("__USE_GPS__", json.dumps(use_gps))
            .replace("__READ_ONLY__", json.dumps(read_only)))


class ViewerHTTPServer(ThreadingHTTPServer):
    """Keep idle browser connections from blocking other clients."""

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(10.0)
        return connection, address


# Matplotlib is shared across request threads; serialize only image rendering.
_IMAGE_RENDER_LOCK = Lock()


class ViewerHandler(BaseHTTPRequestHandler):
    db_path = ""
    cell_size = 0.25
    node_names: dict[str, str] = {}
    use_gps = False
    read_only = True

    def _map_id(self, database: DatabaseManager) -> int:
        query = parse_qs(urlparse(self.path).query)
        if "map_id" in query:
            return int(query["map_id"][0])
        latest = database.latest_map()
        if latest is None:
            raise LookupError("The database contains no maps yet")
        return int(latest["id"])

    def _send(self, body: bytes, content_type: str, filename=None) -> None:
        started = monotonic()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(body)
        self.wfile.flush()
        self.log_message("response sent: %s, %d bytes in %.3fs", self.path, len(body), monotonic() - started)

    def _export_map(self):
        """Download standard map files without writing the database or server disk."""
        origin = self.headers.get('Origin')
        if origin and urlparse(origin).netloc != self.headers.get('Host'):
            self.send_error(403, 'Cross-origin exports are not allowed')
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 1024 or self.headers.get('Content-Type') != 'application/json':
                raise ValueError('Expected a small JSON request')
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict) or type(data.get('map_id')) is not int or data['map_id'] <= 0:
                raise ValueError('Select a valid map')
            name = data.get('name')
            database = DatabaseManager(self.db_path, read_only=True)
            try:
                bundle = export_map_bundle(database, data['map_id'], name)
            finally:
                database.close()
            self._send(bundle, 'application/zip', filename=name+'.zip')
        except (ValueError, TypeError) as error:
            self.send_error(400, str(error))
        except LookupError:
            self.send_error(404, 'Map no longer exists; refresh the viewer')
        except sqlite3.Error:
            self.send_error(503, 'Database unavailable; retry exporting the map')
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def do_POST(self) -> None:
        """Export in either mode; promotion requires writable analysis mode."""
        if urlparse(self.path).path == '/api/export-map':
            self._export_map()
            return
        if self.read_only:
            self.send_error(403, 'Viewer is read-only; restart with --analysis to promote runs')
            return
        if urlparse(self.path).path != '/api/keep-run':
            self.send_error(404)
            return
        origin = self.headers.get('Origin')
        if origin and urlparse(origin).netloc != self.headers.get('Host'):
            self.send_error(403, 'Cross-origin writes are not allowed')
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 1024 or self.headers.get('Content-Type') != 'application/json':
                raise ValueError('Expected a small JSON request')
            data = json.loads(self.rfile.read(length))
            run_id = data.get('run_id') if isinstance(data, dict) else None
            if type(run_id) is not int or run_id < 0:
                raise ValueError('Invalid run ID')
            database = DatabaseManager(self.db_path)
            try:
                found = database.keep_run(run_id)
            finally:
                database.close()
            if not found:
                self.send_error(404, 'Run no longer exists')
                return
            self._send(b'{"kept":true}', 'application/json')
        except (ValueError, TypeError):
            self.send_error(400, 'Invalid keep-run request')
        except sqlite3.Error:
            self.send_error(503, 'Database unavailable; retry keeping the run')
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def do_GET(self) -> None:
        started = monotonic()
        try:
            self._handle_get()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            # A browser can cancel a request during navigation or refresh.
            # The response cannot be delivered; keep serving other requests.
            self.log_message("client disconnected or socket timed out: %s", self.path)
        except LookupError:
            self.send_error(503, "No map has been captured yet; retry after a map is available")
        except sqlite3.Error as error:
            self.log_message("database error for %s: %s", self.path, error)
            try:
                self.send_error(503, "Wi-Fi database temporarily unavailable; retry the request")
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass
        finally:
            self.log_message("request finished: %s in %.3fs", self.path, monotonic() - started)

    def _handle_get(self) -> None:
        path = urlparse(self.path).path
        if path == "/":
            self._send(
                rendered_html(self.node_names, self.use_gps, self.read_only).encode(),
                "text/html; charset=utf-8",
            )
            return
        if path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return
        database = DatabaseManager(self.db_path, read_only=self.read_only)
        try:
            database.conn.execute('BEGIN')
            map_id = self._map_id(database)
            map_data = database.get_map(map_id)
            if map_data is None:
                self.send_error(404, "Map not found")
                return
            if path == "/api/data":
                cells = database.get_cells(map_id)
                # JS numbers cannot represent nanosecond ROS stamps exactly.
                for cell in cells:
                    if cell.get("gps_stamp_ns") is not None:
                        cell["gps_stamp_ns"] = str(cell["gps_stamp_ns"])
                public_map = {
                    key: value for key, value in map_data.items()
                    if key not in ("image_data", "yaml_text", "orientation_yaml")
                }
                run = database.get_run(map_data["run_id"]) if "run_id" in map_data else None
                cell_size = run['cell_size'] if run else self.cell_size
                body = json.dumps(
                    {
                        "maps": database.list_maps(),
                        "run": run,
                        "map": public_map,
                        "orientation": load_saved_orientation(map_data, self.db_path),
                        "cell_size": cell_size,
                        "cells": cells,
                        "surveys": database.get_surveys(map_id),
                        "free_cells": free_cell_centers(map_data, cell_size),
                    }
                ).encode()
                self._send(body, "application/json")
            elif path == "/map.png":
                output = io.BytesIO()
                with _IMAGE_RENDER_LOCK:
                    plt.imsave(
                        output,
                        decode_pgm(map_data["image_data"]),
                        cmap="gray",
                        vmin=0.0,
                        vmax=1.0,
                        format="png",
                    )
                self._send(output.getvalue(), "image/png")
            else:
                self.send_error(404)
        finally:
            database.close()

    def log_message(self, format_string: str, *args) -> None:
        print(f"{self.address_string()} - {format_string % args}")


def snapshot_database(source: str, destination: str) -> None:
    source_path = str(Path(source).expanduser().resolve())
    destination_path = Path(destination).expanduser().resolve()
    destination_path.parent.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(source_path) as source_db:
        with sqlite3.connect(destination_path) as destination_db:
            source_db.backup(destination_db)


def main() -> None:
    parser = argparse.ArgumentParser(description=f"Browse {robot_name()} Wi-Fi maps")
    parser.add_argument("--db", default="~/.ros/titania_wifi.db")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--cell-size", type=float, default=0.25)
    parser.add_argument("--use-gps", action=argparse.BooleanOptionalAction,
                        default=False, help="Display GPS metadata in cell details")
    parser.add_argument(
        "--node-names",
        help="YAML file containing an access_points BSSID-to-name mapping",
        default=None,
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--analysis', action='store_true', default=False,
                      help='Open read/write and enable Promote for post-run analysis')
    mode.add_argument('--read-only', action='store_false', dest='analysis',
                      help='View without database writes (default)')
    parser.add_argument("--snapshot", help="Create a consistent SQLite copy and exit")
    args = parser.parse_args()
    db_path = str(Path(args.db).expanduser().resolve())
    node_names_path = args.node_names
    if not node_names_path and not args.snapshot:
        node_names_path = str(default_node_names_path())
    if node_names_path:
        node_names_path = str(Path(node_names_path).expanduser().resolve())
    snapshot_path = str(Path(args.snapshot).expanduser().resolve()) if args.snapshot else None
    parameters = {
        **vars(args),
        "db": db_path,
        "node_names": node_names_path,
        "snapshot": snapshot_path,
    }
    print("Wi-Fi viewer parameters:\n" + json.dumps(parameters, indent=2), flush=True)
    if args.snapshot:
        snapshot_database(db_path, snapshot_path)
        print(f"Snapshot written to {Path(args.snapshot).expanduser().resolve()}")
        return
    ViewerHandler.db_path = db_path
    ViewerHandler.cell_size = args.cell_size
    ViewerHandler.use_gps = args.use_gps
    ViewerHandler.read_only = not args.analysis
    ViewerHandler.node_names = load_node_names(node_names_path)
    try:
        server = ViewerHTTPServer((args.host, args.port), ViewerHandler)
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        alternative_port = args.port + 1 if args.port < 65535 else 65534
        raise SystemExit(
            f"Cannot start Wi-Fi viewer: {args.host}:{args.port} is already in use.\n"
            "Another viewer may already be running as part of the robot launch. "
            "Stop that viewer, or choose a different port: "
            f"port:={alternative_port} with ros2 launch "
            f"(or --port {alternative_port} with the Python executable)."
        ) from None
    print(f"Wi-Fi viewer: http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
