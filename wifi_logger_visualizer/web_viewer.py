#!/usr/bin/env python3
import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import io
import json
from pathlib import Path
import sqlite3
from urllib.parse import parse_qs, urlparse

import matplotlib.pyplot as plt

from wifi_logger_visualizer.database_manager import DatabaseManager
from wifi_logger_visualizer.map_data import decode_pgm, free_cell_centers


HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>Titania Wi-Fi Map</title>
<style>
body { font-family: sans-serif; margin: 1rem; background: #202124; color: #eee; }
#toolbar { display:flex; gap:1rem; align-items:center; flex-wrap:wrap; margin-bottom:1rem; }
#content { display:flex; gap:1.25rem; align-items:flex-start; }
#wrap { position:relative; display:inline-block; }
#map { display:block; max-width:calc(100vw - 2rem); max-height:75vh; }
#overlay { position:absolute; left:0; top:0; width:100%; height:100%; }
#legend { min-width:10rem; padding:.75rem; background:#303134; border-radius:.4rem; }
#legendTitle { font-weight:bold; margin-bottom:.75rem; }
.legendScale { display:flex; gap:.5rem; height:15rem; }
.legendBar { width:1.5rem; border:1px solid #aaa; }
.legendLabels { display:flex; flex-direction:column; justify-content:space-between; }
.legendEntry { display:flex; gap:.5rem; align-items:center; margin:.4rem 0; }
.legendSwatch { width:1.25rem; height:1.25rem; border:1px solid #aaa; }
#tip { min-height:2rem; } select,button { padding:.3rem; }
</style></head>
<body><h2>Titania Wi-Fi coverage</h2><div id="toolbar">
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
<button id="refresh">Refresh</button><button id="export">Export PNG</button></div>
<div id="content"><div id="wrap"><img id="map"><canvas id="overlay"></canvas></div>
<div id="legend"><div id="legendTitle"></div><div id="legendBody"></div></div></div>
<div id="tip"></div>
<script>
const ranges={signal:[-90,-35],tx:[0,1200],rx:[0,1200],upload:[0,500],
 download:[0,500],idle_latency:[0,100],loaded_latency:[0,200],age:[0,30]};
const titles={signal:'Signal strength',tx:'TX link rate',rx:'RX link rate',
 upload:'Upload to server',download:'Download from server',
 idle_latency:'Idle latency',loaded_latency:'Loaded latency',
 age:'Observation age',bssid:'Access point'};
const units={signal:'dBm',tx:'Mb/s',rx:'Mb/s',upload:'Mb/s',download:'Mb/s',
 idle_latency:'ms',loaded_latency:'ms',age:'days'};
const nodeNames={'24:e5:0f:02:ac:1a':'Living Room','24:e5:0f:02:b5:89':'Dining Room',
 '24:e5:0f:43:50:53':'Music Room','b8:7b:d4:b9:81:8f':'Computer Room'};
function apLabel(bssid){
 if(!bssid)return 'Unknown Node';
 return (nodeNames[bssid.toLowerCase()]||'Unknown Node')+' ('+bssid+')';
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
  swatch.style.background='rgba(90,90,90,.28)';
  const label=document.createElement('span');label.textContent='Unmeasured free space';
  note.append(swatch,label);body.append(note);}
}
function draw(){
 const img=document.getElementById('map'), c=document.getElementById('overlay');
 c.width=img.naturalWidth;c.height=img.naturalHeight;const x=c.getContext('2d');
 x.clearRect(0,0,c.width,c.height);drawn=[];
 const metric=document.getElementById('metric').value;
 drawLegend(metric);
 const size=payload.cell_size/payload.map.resolution;
 const measured=new Set(payload.cells.map(v=>v.cell_x+','+v.cell_y));
 if(document.getElementById('unmeasured').checked){
  x.fillStyle='rgba(90,90,90,.28)';
  for(const p of payload.free_cells){const [px,py]=pixel(p[0],p[1]);
   const key=Math.floor(p[0]/payload.cell_size)+','+Math.floor(p[1]/payload.cell_size);
   if(!measured.has(key))x.fillRect(px-size/2,py-size/2,size,size);}
 }
 for(const cell of payload.cells){const v=valueFor(cell,metric);
  if(v===null||v===undefined)continue;
  const [px,py]=pixel(cell.center_x,cell.center_y);x.fillStyle=color(v,metric);
  x.fillRect(px-size/2,py-size/2,size,size);drawn.push({px,py,size,cell,value:v});}
}
async function load(){
 const id=document.getElementById('mapSelect').value;
 payload=await (await fetch('/api/data'+(id?'?map_id='+id:''))).json();
 const sel=document.getElementById('mapSelect');if(!sel.options.length){
  for(const m of payload.maps){const o=document.createElement('option');o.value=m.id;
   o.text=`${m.id}: ${m.source_yaml_path} (${m.content_hash.slice(0,8)})`;sel.add(o);}
  sel.value=payload.map.id;
 }
 const img=document.getElementById('map');img.onload=draw;
 img.src='/map.png?map_id='+payload.map.id+'&t='+Date.now();
}
document.getElementById('metric').onchange=draw;
document.getElementById('unmeasured').onchange=draw;
document.getElementById('mapSelect').onchange=load;
document.getElementById('refresh').onclick=load;
document.getElementById('export').onclick=()=>{
 const image=document.getElementById('map'),overlay=document.getElementById('overlay');
 const output=document.createElement('canvas');output.width=overlay.width;
 output.height=overlay.height;const context=output.getContext('2d');
 context.drawImage(image,0,0);context.drawImage(overlay,0,0);
 const link=document.createElement('a');link.download='titania-wifi-map.png';
 link.href=output.toDataURL('image/png');link.click();};
document.getElementById('overlay').onmousemove=e=>{const r=e.target.getBoundingClientRect();
 const px=(e.clientX-r.left)*e.target.width/r.width,py=(e.clientY-r.top)*e.target.height/r.height;
 const hit=drawn.find(v=>Math.abs(v.px-px)<v.size/2&&Math.abs(v.py-py)<v.size/2);
 const position=hit?
  `(${hit.cell.center_x.toFixed(2)}, ${hit.cell.center_y.toFixed(2)})`:'';
 const signal=hit?
  `signal min/last/max ${hit.cell.signal_dbm_min}/`+
  `${hit.cell.signal_dbm_last}/${hit.cell.signal_dbm_max} dBm`:'';
 const metric=document.getElementById('metric').value;
 const onAp=metric==='bssid';
 const shown=hit?(onAp?apLabel(hit.value):hit.value):'';
 const node=onAp?'':`${apLabel(hit?hit.cell.bssid:null)}; `;
 document.getElementById('tip').textContent=hit?
  `${shown} at ${position}; ${signal}; ${node}`+
  hit.cell.last_observed_at:'';};
load();
</script></body></html>"""


class ViewerHandler(BaseHTTPRequestHandler):
    db_path = ""
    cell_size = 0.25

    def _map_id(self, database: DatabaseManager) -> int:
        query = parse_qs(urlparse(self.path).query)
        if "map_id" in query:
            return int(query["map_id"][0])
        latest = database.latest_map()
        if latest is None:
            raise RuntimeError("The database contains no maps")
        return int(latest["id"])

    def _send(self, body: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/":
            self._send(HTML.encode(), "text/html; charset=utf-8")
            return
        database = DatabaseManager(self.db_path)
        try:
            map_id = self._map_id(database)
            map_data = database.get_map(map_id)
            if map_data is None:
                self.send_error(404, "Map not found")
                return
            if path == "/api/data":
                public_map = {
                    key: value for key, value in map_data.items()
                    if key not in ("image_data", "yaml_text")
                }
                body = json.dumps(
                    {
                        "maps": database.list_maps(),
                        "map": public_map,
                        "cell_size": self.cell_size,
                        "cells": database.get_cells(map_id),
                        "surveys": database.get_surveys(map_id),
                        "free_cells": free_cell_centers(map_data, self.cell_size),
                    }
                ).encode()
                self._send(body, "application/json")
            elif path == "/map.png":
                output = io.BytesIO()
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
    parser = argparse.ArgumentParser(description="Browse Titania Wi-Fi maps")
    parser.add_argument("--db", default="~/.ros/titania_wifi.db")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--cell-size", type=float, default=0.25)
    parser.add_argument("--snapshot", help="Create a consistent SQLite copy and exit")
    args = parser.parse_args()
    db_path = str(Path(args.db).expanduser().resolve())
    if args.snapshot:
        snapshot_database(db_path, args.snapshot)
        print(f"Snapshot written to {Path(args.snapshot).expanduser().resolve()}")
        return
    ViewerHandler.db_path = db_path
    ViewerHandler.cell_size = args.cell_size
    server = HTTPServer((args.host, args.port), ViewerHandler)
    print(f"Wi-Fi viewer: http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
