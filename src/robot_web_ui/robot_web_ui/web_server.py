"""
Web server node – FastAPI + WebSocket bridge to ROS2.
Unified UI: SLAM mapping, save/load maps, Nav2 navigation, manual control.
"""
import asyncio
import json
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

import yaml


class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')

        self.declare_parameter('port', 8080)
        self.declare_parameter('rooms_file', '')

        self._port = self.get_parameter('port').value
        self._rooms = {}
        self._status = {}
        self._map_data = None
        self._map_version = 0
        self._lock = threading.Lock()

        rooms_file = self.get_parameter('rooms_file').value
        if rooms_file and os.path.isfile(rooms_file):
            with open(rooms_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                self._rooms = data.get('rooms', {})

        self._cmd_pub = self.create_publisher(
            String, '/robot_web_ui/command', 10)
        self.create_subscription(
            String, '/robot_web_ui/status', self._status_cb, 10)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)

        self.get_logger().info(f'Web server on port {self._port}')

    def _status_cb(self, msg):
        try:
            with self._lock:
                self._status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _map_cb(self, msg):
        info = msg.info
        with self._lock:
            self._map_version += 1
            self._map_data = {
                'version': self._map_version,
                'width': info.width,
                'height': info.height,
                'resolution': info.resolution,
                'origin_x': info.origin.position.x,
                'origin_y': info.origin.position.y,
                'data': list(msg.data),
            }

    def publish_command(self, cmd_dict):
        msg = String()
        msg.data = json.dumps(cmd_dict)
        self._cmd_pub.publish(msg)

    def get_status(self):
        with self._lock:
            return dict(self._status)

    def get_map(self):
        with self._lock:
            return self._map_data

    def get_map_version(self):
        with self._lock:
            return self._map_version

    def get_rooms(self):
        return self._rooms


def create_app(ros_node: WebServerNode) -> FastAPI:
    app = FastAPI(title='Robot Navigation UI')

    @app.get('/')
    async def index():
        return HTMLResponse(get_index_html())

    @app.get('/api/rooms')
    async def get_rooms():
        return ros_node.get_rooms()

    @app.get('/api/status')
    async def get_status():
        return ros_node.get_status()

    @app.websocket('/ws')
    async def websocket_endpoint(ws: WebSocket):
        await ws.accept()
        ros_node.get_logger().info('WebSocket connected')

        rooms = ros_node.get_rooms()
        await ws.send_json({'type': 'rooms', 'data': rooms})

        last_map_version = 0
        map_data = ros_node.get_map()
        if map_data:
            await ws.send_json({'type': 'map', 'data': map_data})
            last_map_version = map_data.get('version', 0)

        loop_count = 0
        try:
            while True:
                try:
                    raw = await asyncio.wait_for(ws.receive_text(), timeout=0.2)
                    cmd = json.loads(raw)
                    ros_node.publish_command(cmd)
                except asyncio.TimeoutError:
                    pass

                status = ros_node.get_status()
                if status:
                    await ws.send_json({'type': 'status', 'data': status})

                loop_count += 1
                if loop_count >= 15:
                    loop_count = 0
                    cur_ver = ros_node.get_map_version()
                    if cur_ver > last_map_version:
                        map_data = ros_node.get_map()
                        if map_data:
                            last_map_version = cur_ver
                            await ws.send_json({'type': 'map', 'data': map_data})

        except WebSocketDisconnect:
            ros_node.get_logger().info('WebSocket disconnected')
        except Exception as e:
            ros_node.get_logger().warn(f'WebSocket error: {e}')

    return app


def get_index_html():
    return '''<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Robot Navigation Control</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#0f172a;color:#e2e8f0;height:100vh;overflow:hidden}
.layout{display:grid;grid-template-columns:340px 1fr 320px;grid-template-rows:60px 1fr;height:100vh}
.header{grid-column:1/-1;background:#1e293b;display:flex;align-items:center;padding:0 24px;border-bottom:1px solid #334155;justify-content:space-between}
.header h1{font-size:1.2rem;color:#38bdf8}
.badge{padding:5px 14px;border-radius:20px;font-size:.8rem;font-weight:700;text-transform:uppercase;margin-left:8px}
.mode-idle{background:#334155;color:#94a3b8}
.mode-slam{background:#7c3aed;color:#e9d5ff}
.mode-nav{background:#0d9488;color:#ccfbf1}
.nav-idle{background:#334155;color:#94a3b8}
.nav-navigating{background:#1e3a5f;color:#38bdf8;animation:pulse 1.5s infinite}
.nav-succeeded{background:#14532d;color:#4ade80}
.nav-failed{background:#7f1d1d;color:#f87171}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.6}}

.sidebar{background:#1e293b;overflow-y:auto;padding:16px;border-right:1px solid #334155}
.sidebar-right{background:#1e293b;overflow-y:auto;padding:16px;border-left:1px solid #334155}
.section{margin-bottom:20px}
.section-title{font-size:.7rem;text-transform:uppercase;letter-spacing:.1em;color:#64748b;margin-bottom:10px;font-weight:700}

.btn{display:block;width:100%;padding:10px;border:1px solid #334155;border-radius:8px;background:#0f172a;color:#e2e8f0;font-size:.85rem;cursor:pointer;transition:all .2s;text-align:center;margin-bottom:6px}
.btn:hover:not(:disabled){border-color:#38bdf8;background:#1a2744}
.btn:disabled{opacity:.4;cursor:not-allowed}
.btn-purple{border-color:#7c3aed;color:#c4b5fd}
.btn-purple:hover:not(:disabled){background:#2e1065;border-color:#a78bfa}
.btn-teal{border-color:#0d9488;color:#5eead4}
.btn-teal:hover:not(:disabled){background:#042f2e;border-color:#2dd4bf}
.btn-red{border-color:#991b1b;color:#fca5a5}
.btn-red:hover:not(:disabled){background:#450a0a;border-color:#f87171}
.btn-blue{border-color:#1d4ed8;color:#93c5fd}
.btn-blue:hover:not(:disabled){background:#172554;border-color:#60a5fa}

.input-row{display:flex;gap:6px;margin-bottom:6px}
.input-row input,.input-row select{flex:1;padding:8px;border:1px solid #334155;border-radius:8px;background:#0f172a;color:#e2e8f0;font-size:.85rem}
.input-row select{cursor:pointer}

.room-card{background:#0f172a;border:1px solid #334155;border-radius:10px;padding:12px;margin-bottom:8px;cursor:pointer;transition:all .2s;display:flex;align-items:center;gap:10px}
.room-card:hover{border-color:#38bdf8;background:#1a2744}
.room-card.active{border-color:#38bdf8;background:#1e3a5f}
.room-icon{font-size:1.6rem}
.room-info h3{font-size:.9rem;margin-bottom:1px}
.room-info p{font-size:.7rem;color:#64748b}

.map-container{position:relative;display:flex;align-items:center;justify-content:center;background:#0a0f1c;overflow:hidden}
#mapCanvas{border:1px solid #334155;border-radius:8px}

.joystick-container{width:150px;height:150px;background:#0f172a;border-radius:50%;border:2px solid #334155;margin:0 auto;position:relative;touch-action:none}
.joystick-knob{width:46px;height:46px;background:#38bdf8;border-radius:50%;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);cursor:grab}
.joystick-knob:active{cursor:grabbing;box-shadow:0 0 20px rgba(56,189,248,.5)}
.cross{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);width:100%;height:100%;pointer-events:none}
.cross::before,.cross::after{content:'';position:absolute;background:#334155}
.cross::before{width:1px;height:100%;left:50%}
.cross::after{height:1px;width:100%;top:50%}

.info-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.info-item{background:#0f172a;border:1px solid #334155;border-radius:8px;padding:8px;text-align:center}
.info-item .label{font-size:.65rem;color:#64748b;text-transform:uppercase}
.info-item .value{font-size:1rem;font-weight:700;color:#38bdf8;margin-top:1px}

.conn-dot{width:10px;height:10px;border-radius:50%;display:inline-block;margin-right:8px}
.connected{background:#4ade80}
.disconnected{background:#f87171}

.log-box{background:#0f172a;border:1px solid #334155;border-radius:8px;padding:8px;font-size:.75rem;color:#94a3b8;max-height:80px;overflow-y:auto;margin-top:6px;font-family:monospace}
</style>
</head>
<body>
<div class="layout">
    <div class="header">
        <div style="display:flex;align-items:center;gap:10px">
            <h1>Robot Navigation</h1>
            <span class="conn-dot disconnected" id="connDot"></span>
            <span class="badge mode-idle" id="modeBadge">IDLE</span>
        </div>
        <div>
            <span class="badge nav-idle" id="navBadge">-</span>
            <span id="goalLabel" style="margin-left:8px;font-size:.85rem;color:#94a3b8"></span>
        </div>
    </div>

    <!-- LEFT SIDEBAR: Mode controls + rooms -->
    <div class="sidebar">
        <!-- SLAM Section -->
        <div class="section">
            <div class="section-title">1. Quet ban do (SLAM)</div>
            <button class="btn btn-purple" id="btnStartSlam" onclick="startSlam()">Bat dau quet map</button>
            <div class="input-row">
                <input type="text" id="mapNameInput" placeholder="Ten ban do..." value="hospital">
                <button class="btn btn-blue" id="btnSaveMap" onclick="saveMap()" disabled style="flex:0 0 auto;width:auto;padding:10px 16px">Luu</button>
            </div>
            <button class="btn btn-red" id="btnStopSlam" onclick="stopSlam()" disabled>Dung quet map</button>
        </div>

        <!-- Nav Section -->
        <div class="section">
            <div class="section-title">2. Tu hanh (Navigation)</div>
            <div class="input-row">
                <select id="mapSelect"><option value="">-- Chon ban do --</option></select>
            </div>
            <button class="btn btn-teal" id="btnStartNav" onclick="startNav()">Bat dau tu hanh</button>
            <button class="btn btn-red" id="btnStopNav" onclick="stopNav()" disabled>Dung tu hanh</button>
        </div>

        <!-- Room list -->
        <div class="section" id="roomSection" style="display:none">
            <div class="section-title">3. Chon diem den</div>
            <div id="roomList"></div>
        </div>

        <div class="section">
            <div class="section-title">Huong dan</div>
            <div class="log-box" id="logBox">San sang. Chon che do de bat dau.</div>
        </div>
    </div>

    <!-- MAP -->
    <div class="map-container" id="mapContainer">
        <canvas id="mapCanvas" width="800" height="600"></canvas>
    </div>

    <!-- RIGHT SIDEBAR: Controls -->
    <div class="sidebar-right">
        <div class="section">
            <div class="section-title">Dieu khien tay (WASD)</div>
            <div class="joystick-container" id="joystick">
                <div class="cross"></div>
                <div class="joystick-knob" id="joystickKnob"></div>
            </div>
            <div style="display:flex;gap:6px;margin-top:10px">
                <button class="btn btn-red" onclick="sendStop()">DUNG</button>
                <button class="btn btn-red" onclick="cancelNav()">HUY NAV</button>
            </div>
        </div>

        <div class="section">
            <div class="section-title">Vi tri Robot</div>
            <div class="info-grid">
                <div class="info-item"><div class="label">X</div><div class="value" id="posX">0.00</div></div>
                <div class="info-item"><div class="label">Y</div><div class="value" id="posY">0.00</div></div>
                <div class="info-item"><div class="label">Yaw</div><div class="value" id="posYaw">0.0</div></div>
                <div class="info-item"><div class="label">Mode</div><div class="value" id="modeVal">-</div></div>
            </div>
        </div>

        <div class="section">
            <div class="section-title">Toc do</div>
            <label style="font-size:.78rem;color:#94a3b8">Tuyen tinh: <span id="linLbl">0.3</span> m/s</label>
            <input type="range" id="linSpeed" min="0.1" max="1.0" step="0.05" value="0.3" style="width:100%" oninput="document.getElementById('linLbl').textContent=this.value">
            <label style="font-size:.78rem;color:#94a3b8">Xoay: <span id="angLbl">0.5</span> rad/s</label>
            <input type="range" id="angSpeed" min="0.1" max="2.0" step="0.1" value="0.5" style="width:100%" oninput="document.getElementById('angLbl').textContent=this.value">
        </div>
    </div>
</div>

<script>
let ws=null, rooms={}, mapInfo=null, mapImageData=null;
let robotPose={x:0,y:0,yaw:0}, selectedRoom=null, currentMode='idle', currentPath=[];
const canvas=document.getElementById('mapCanvas'), ctx=canvas.getContext('2d');

function log(msg){const b=document.getElementById('logBox');b.innerHTML=msg+'<br>'+b.innerHTML;if(b.children.length>20)b.lastChild.remove()}

// ===== WebSocket =====
function connect(){
    const proto=location.protocol==='https:'?'wss':'ws';
    ws=new WebSocket(proto+'://'+location.host+'/ws');
    ws.onopen=()=>{document.getElementById('connDot').className='conn-dot connected';log('Da ket noi server')};
    ws.onclose=()=>{document.getElementById('connDot').className='conn-dot disconnected';setTimeout(connect,2000)};
    ws.onmessage=(e)=>{const m=JSON.parse(e.data);
        if(m.type==='rooms')handleRooms(m.data);
        else if(m.type==='status')handleStatus(m.data);
        else if(m.type==='map')handleMap(m.data)};
}
connect();
function send(o){if(ws&&ws.readyState===1)ws.send(JSON.stringify(o))}

// ===== Mode Controls =====
function startSlam(){send({action:'start_slam'});log('Dang bat dau quet map...')}
function stopSlam(){send({action:'stop_slam'});log('Dang dung quet map...')}
function saveMap(){
    const n=document.getElementById('mapNameInput').value.trim();
    if(!n){alert('Nhap ten ban do');return}
    send({action:'save_map',name:n});log('Dang luu ban do: '+n+'...')
}
function startNav(){
    const m=document.getElementById('mapSelect').value;
    if(!m){alert('Chon ban do truoc');return}
    send({action:'start_nav',map:m});log('Dang khoi dong tu hanh voi map: '+m)
}
function stopNav(){send({action:'stop_nav'});log('Dang dung tu hanh...')}

function updateUI(mode){
    currentMode=mode;
    const mb=document.getElementById('modeBadge');
    mb.textContent=mode.toUpperCase();
    mb.className='badge mode-'+mode;
    document.getElementById('modeVal').textContent=mode;

    const isIdle=mode==='idle', isSlam=mode==='slam', isNav=mode==='nav';
    document.getElementById('btnStartSlam').disabled=!isIdle;
    document.getElementById('btnStopSlam').disabled=!isSlam;
    document.getElementById('btnSaveMap').disabled=!isSlam;
    document.getElementById('btnStartNav').disabled=!isIdle;
    document.getElementById('btnStopNav').disabled=!isNav;
    document.getElementById('roomSection').style.display=isNav?'block':'none';
}

// ===== Status =====
function handleStatus(d){
    if(d.pose){
        robotPose=d.pose;
        document.getElementById('posX').textContent=d.pose.x.toFixed(2);
        document.getElementById('posY').textContent=d.pose.y.toFixed(2);
        document.getElementById('posYaw').textContent=(d.pose.yaw*180/Math.PI).toFixed(1)+'°';
    }
    if(d.nav_status){
        const b=document.getElementById('navBadge');
        b.textContent=d.nav_status.toUpperCase();
        b.className='badge nav-'+d.nav_status;
    }
    if(d.goal_name)document.getElementById('goalLabel').textContent=d.goal_name?'-> '+d.goal_name:'';
    if(d.mode)updateUI(d.mode);
    if(d.available_maps){
        const sel=document.getElementById('mapSelect');
        const cur=sel.value;
        sel.innerHTML='<option value="">-- Chon ban do --</option>';
        d.available_maps.forEach(m=>{const o=document.createElement('option');o.value=m;o.textContent=m;sel.appendChild(o)});
        if(cur)sel.value=cur;
    }
    if(d.path !== undefined) currentPath = d.path;
    drawMap();
}

// ===== Rooms =====
function handleRooms(data){
    rooms=data;
    const list=document.getElementById('roomList');
    list.innerHTML='';
    for(const[id,r]of Object.entries(rooms)){
        const c=document.createElement('div');c.className='room-card';c.dataset.id=id;
        c.innerHTML='<div class="room-icon">'+(r.icon||'📍')+'</div><div class="room-info"><h3>'+r.name+'</h3><p>x:'+r.x.toFixed(1)+' y:'+r.y.toFixed(1)+'</p></div>';
        c.onclick=()=>navToRoom(id);list.appendChild(c);
    }
}
function navToRoom(id){
    if(currentMode!=='nav'){log('Phai o che do TU HANH moi dieu huong duoc');return}
    const r=rooms[id];if(!r)return;
    selectedRoom=id;
    document.querySelectorAll('.room-card').forEach(c=>c.classList.remove('active'));
    document.querySelector('.room-card[data-id="'+id+'"]')?.classList.add('active');
    send({action:'navigate',x:r.x,y:r.y,yaw:r.yaw||0,name:r.name});
    log('Di den: '+r.name);
}

// ===== Map Drawing =====
function handleMap(data){
    mapInfo={width:data.width,height:data.height,resolution:data.resolution,origin_x:data.origin_x,origin_y:data.origin_y};
    const img=ctx.createImageData(data.width,data.height);
    for(let i=0;i<data.data.length;i++){
        const v=data.data[i];let r,g,b;
        if(v===-1||v===255){r=40;g=44;b=52}
        else if(v===0){r=20;g=27;b=45}
        else{r=200;g=60;b=60}
        const idx=i*4;img.data[idx]=r;img.data[idx+1]=g;img.data[idx+2]=b;img.data[idx+3]=255;
    }
    mapImageData=img;drawMap();
}

function drawMap(){
    const ct=document.getElementById('mapContainer');
    canvas.width=ct.clientWidth-20;canvas.height=ct.clientHeight-20;
    ctx.fillStyle='#0a0f1c';ctx.fillRect(0,0,canvas.width,canvas.height);

    if(!mapImageData||!mapInfo){
        ctx.fillStyle='#64748b';ctx.font='16px sans-serif';ctx.textAlign='center';
        ctx.fillText('Chua co ban do - Hay chon "Bat dau quet map"',canvas.width/2,canvas.height/2);
        return;
    }
    const sx=canvas.width/mapInfo.width,sy=canvas.height/mapInfo.height,sc=Math.min(sx,sy)*.9;
    const ox=(canvas.width-mapInfo.width*sc)/2,oy=(canvas.height-mapInfo.height*sc)/2;

    const oc=document.createElement('canvas');oc.width=mapInfo.width;oc.height=mapInfo.height;
    oc.getContext('2d').putImageData(mapImageData,0,0);
    ctx.save();ctx.translate(ox,oy);ctx.scale(sc,sc);ctx.translate(0,mapInfo.height);ctx.scale(1,-1);
    ctx.drawImage(oc,0,0);ctx.restore();

    function w2c(wx,wy){
        const mx=(wx-mapInfo.origin_x)/mapInfo.resolution,my=(wy-mapInfo.origin_y)/mapInfo.resolution;
        return{x:ox+mx*sc,y:oy+(mapInfo.height-my)*sc};
    }

    // Room markers (only in nav mode)
    if(currentMode==='nav'){
        for(const[id,r]of Object.entries(rooms)){
            const p=w2c(r.x,r.y);
            ctx.beginPath();ctx.arc(p.x,p.y,8,0,Math.PI*2);
            ctx.fillStyle=id===selectedRoom?'#38bdf8':'#fbbf24';ctx.fill();
            ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();
            ctx.fillStyle='#fff';ctx.font='bold 11px sans-serif';ctx.textAlign='center';
            ctx.fillText(r.name,p.x,p.y-14);
        }
    }

    // Robot
    if (currentMode === 'nav' && currentPath && currentPath.length > 0) {
        ctx.beginPath();
        ctx.strokeStyle = '#4ade80'; 
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);     
        
        for (let i = 0; i < currentPath.length; i++) {
            const p = w2c(currentPath[i].x, currentPath[i].y);
            if (i === 0) ctx.moveTo(p.x, p.y);
            else ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
        ctx.setLineDash([]); // Reset lại nét vẽ bình thường cho các hình khác
    }
    const rp=w2c(robotPose.x,robotPose.y);
    ctx.save();ctx.translate(rp.x,rp.y);ctx.rotate(-robotPose.yaw);
    ctx.beginPath();ctx.arc(0,0,10,0,Math.PI*2);ctx.fillStyle='#4ade80';ctx.fill();
    ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();
    ctx.beginPath();ctx.moveTo(14,0);ctx.lineTo(5,-6);ctx.lineTo(5,6);ctx.closePath();
    ctx.fillStyle='#4ade80';ctx.fill();ctx.restore();
}

// ===== Map click =====
canvas.addEventListener('click',(e)=>{
    if(!mapInfo||currentMode!=='nav')return;
    const rect=canvas.getBoundingClientRect(),cx=e.clientX-rect.left,cy=e.clientY-rect.top;
    const sx=canvas.width/mapInfo.width,sy=canvas.height/mapInfo.height,sc=Math.min(sx,sy)*.9;
    const ox=(canvas.width-mapInfo.width*sc)/2,oy=(canvas.height-mapInfo.height*sc)/2;
    const mx=(cx-ox)/sc,my=mapInfo.height-(cy-oy)/sc;
    const wx=mx*mapInfo.resolution+mapInfo.origin_x,wy=my*mapInfo.resolution+mapInfo.origin_y;
    selectedRoom=null;
    document.querySelectorAll('.room-card').forEach(c=>c.classList.remove('active'));
    send({action:'navigate',x:wx,y:wy,yaw:0,name:'Custom ('+wx.toFixed(1)+', '+wy.toFixed(1)+')'});
    log('Di den: ('+wx.toFixed(1)+', '+wy.toFixed(1)+')');
});

// ===== Joystick =====
const joystick=document.getElementById('joystick'),knob=document.getElementById('joystickKnob');
let dragging=false;
function handleJ(e){
    if(!dragging)return;e.preventDefault();
    const r=joystick.getBoundingClientRect(),cx2=r.left+r.width/2,cy2=r.top+r.height/2;
    const cX=e.touches?e.touches[0].clientX:e.clientX,cY=e.touches?e.touches[0].clientY:e.clientY;
    let dx=cX-cx2,dy=cY-cy2;const mR=r.width/2-23,dist=Math.sqrt(dx*dx+dy*dy);
    if(dist>mR){dx=dx/dist*mR;dy=dy/dist*mR}
    knob.style.left=(r.width/2+dx)+'px';knob.style.top=(r.height/2+dy)+'px';
    const lm=parseFloat(document.getElementById('linSpeed').value);
    const am=parseFloat(document.getElementById('angSpeed').value);
    send({action:'cmd_vel',linear:-dy/mR*lm,angular:-dx/mR*am});
}
function stopJ(){dragging=false;knob.style.left='50%';knob.style.top='50%';knob.style.transform='translate(-50%,-50%)';send({action:'stop'})}
knob.addEventListener('mousedown',()=>{dragging=true});
knob.addEventListener('touchstart',()=>{dragging=true});
document.addEventListener('mousemove',handleJ);document.addEventListener('touchmove',handleJ);
document.addEventListener('mouseup',stopJ);document.addEventListener('touchend',stopJ);

function sendStop(){send({action:'stop'})}
function cancelNav(){send({action:'cancel'})}

// ===== Keyboard =====
const ks={};
document.addEventListener('keydown',(e)=>{if(e.target.tagName==='INPUT'||e.target.tagName==='SELECT')return;if(ks[e.key])return;ks[e.key]=true;sendKV()});
document.addEventListener('keyup',(e)=>{delete ks[e.key];sendKV()});
function sendKV(){
    const l=parseFloat(document.getElementById('linSpeed').value),a=parseFloat(document.getElementById('angSpeed').value);
    let li=0,an=0;
    if(ks['ArrowUp']||ks['w'])li+=l;if(ks['ArrowDown']||ks['s'])li-=l;
    if(ks['ArrowLeft']||ks['a'])an+=a;if(ks['ArrowRight']||ks['d'])an-=a;
    if(li===0&&an===0)send({action:'stop'});else send({action:'cmd_vel',linear:li,angular:an});
}
window.addEventListener('resize',drawMap);
</script>
</body>
</html>'''


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    app = create_app(node)
    try:
        uvicorn.run(app, host='0.0.0.0', port=node._port, log_level='info')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
