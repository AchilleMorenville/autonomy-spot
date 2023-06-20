from typing import Union
import os
import sys
import datetime
import json

import rclpy

from fastapi import FastAPI, BackgroundTasks, Response, status
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse
from fastapi.responses import FileResponse
from fastapi.responses import HTMLResponse

from export import *

# from tb_rest_client.rest_client_pe import Device, RestClientPE
# from tb_rest_client.rest import ApiException

from ros_interface import RosInterface

description = """
Spot 3D scan backend API helps you do awesome stuff using Spot. 🚀

## Amazing stuff 1

***************

## Amazing stuf 2

* blabla 
* labla
"""

app = FastAPI(
    title="Spot 3D scan backend",
    description=description,
    version="1.0.0",
    terms_of_service="http://example.com/terms/",
    contact={
        "name": "Amazing R&D team",
        "url": "https://buildwise.be",
        "email": "user@buildwise.be",
    },
    license_info={
        "name": "A-GPL",
        "url": "https://www.gnu.org/licenses/agpl-3.0.html",
    },
)

app.mount("/static", StaticFiles(directory="static"), name="static")


TB_user = 'dumy@dumy.com'
TB_password = 'Dumy123!'
TB_url = "https://dashboard.digitalconstruction.cloud"
SITE3D_url = "https://site3d.digitalconstruction.cloud"

favicon_path = 'static/favicon.ico'

ros_interface = None

@app.get('/favicon.ico', include_in_schema=False)
async def favicon():
    return FileResponse(favicon_path)


@app.get("/", response_class=HTMLResponse)
async def read_items():
    return """
    <html>
        <head>
            <title>Welcom to SPOT BACKEND REST API</title>
        </head>
        <body>
            <h1>\n\n\nplease look at /docs or /redoc for detailed information</h1>
        </body>
    </html>
    """
def wakeup():
    status = False
    out = subprocess.run(['ptpcam', '--set-property=0xD80E' ,'--val=0x00'], capture_output=True, text=True)
    if ('succeeded' in out.stdout) : status = True
    return status

@app.get("/set_360_wakeup")
def set_360_wakeup(response: Response):
    try:
        status = wakeup()
        return {"msg" : status}
    except Exception as e:
        return {"msg" : str(e)}

@app.get("/get_360_image")
def get_360_image(response: Response):
    try:
        import subprocess
        import os
        import glob
        import shutil
        from datetime import datetime
        import math
        import json
        import time
        #attempt = 0
        #while (not wakeup()) :
        #   attempt+=1
        #   if (attempt > 10 ) : return {'msg' : "360 degree cam does'nt wake up!!\n max num of attempts reached!"}
        #time.sleep(0.5)
        path = "/data/360"
        os.chdir(path)

        #os.mkdir("im")
        ts = math.modf(datetime.timestamp(datetime.now()))
        out = subprocess.run(['gphoto2', '--capture-image-and-download'], capture_output=True, text=True)
        l = glob.glob('*.JPG')
        shutil.move(l[0], 'im')

        data = {}
        data['im360'] = l[0]
        data['ts'] ={}
        data['ts']['sec'] = math.floor(ts[1])
        data['ts']['nanosec'] = math.floor(ts[0]*1e9)
        json_file = 'im/' + l[0].split('.JPG')[0] + '.json'
        with open(json_file, "w") as write_file:
            json.dump(data, write_file, indent=4)
        
        return {"msg" : str(out.stdout)}

    except Exception as e:
        print(str(e))
        return {'msg' : str(e)}


@app.get("/launch") 
def launch(response: Response):
    global ros_interface
    try:
        if ros_interface is None:
            rclpy.init()
            ros_interface = RosInterface()
            return {"msg" : "Ros interface launched"}
        else:
            return {"msg" : "Ros interface already launched"}

    except Exception as e:
        print(str(e))
        return {'msg' : str(e)}

@app.get("/stop") 
def stop(response: Response):
    global ros_interface  # Ne sais absolument pas pourquoi je dois faire ça ici alors que sur l'ancien pas besoin
    try:
        if ros_interface is not None:
            ros_interface.destroy_node()
            ros_interface = None
            rclpy.shutdown()
            return {"msg" : "Ros interface stopped and destroyed"}
        else:
            return {"msg" : "Ros already stopped and destroyed"}
    except Exception as e:
        print(str(e))
        return {'msg' : str(e)}


@app.get("/connect2Spot", status_code=200)
def connect2spot(response: Response):
    global ros_interface
    try :
        if ros_interface is None:
            return JSONResponse(status_code=400, content={'msg' : "Ros interface not launched"}) 

        ret = ros_interface.send_request_spot_driver_connect()
        if ret is None : 
            return JSONResponse(status_code=400, content={'msg' : "Ros service unreachable"}) 
        elif not ret.success :
            return JSONResponse(status_code=400, content={'msg' : "Can't connect to Spot : " + ret.message}) 
        else : 
            return {'msg' : "Connected to Spot"}
    except Exception as e : 
        print(str(e))
        return JSONResponse(status_code=400, content={'msg' : str(e)}) 

@app.get("/start_slam") 
def start_slam(response: Response):
    global ros_interface
    try:
        if ros_interface is None:
            return JSONResponse(status_code=400, content={'msg' : "Ros interface not launched"}) 

        ret = ros_interface.send_request_slam_start()
        if ret is None : 
            return JSONResponse(status_code=400, content={'msg' : "Ros service unreachable"}) 
        elif not ret.success :
            return JSONResponse(status_code=400, content={'msg' : "Can't start slam : " + ret.message}) 
        else : 
            return {'msg' : "Slam started"}

    except Exception as e:
        print(str(e))
        return {'msg' : str(e)}

@app.get("/stop_slam") 
def stop_slam(response: Response):
    global ros_interface
    try:
        if ros_interface is None:
            return JSONResponse(status_code=400, content={'msg' : "Ros interface not launched"}) 

        ret = ros_interface.send_request_slam_stop()
        if ret is None : 
            return JSONResponse(status_code=400, content={'msg' : "Ros service unreachable"}) 
        elif not ret.success :
            return JSONResponse(status_code=400, content={'msg' : "Can't stop slam : " + ret.message}) 
        else : 
            return {'msg' : "Slam stoped"}

    except Exception as e:
        print(str(e))
        return {'msg' : str(e)}

@app.get("/reset_slam") 
def reset_slam(response: Response):
    global ros_interface
    try:
        if ros_interface is None:
            return JSONResponse(status_code=400, content={'msg' : "Ros interface not launched"}) 

        ret = ros_interface.send_request_slam_reset()
        if ret is None : 
            return JSONResponse(status_code=400, content={'msg' : "Ros service unreachable"}) 
        elif not ret.success :
            return JSONResponse(status_code=400, content={'msg' : "Can't reset slam : " + ret.message}) 
        else : 
            return {'msg' : "Slam reset"}

    except Exception as e:
        print(str(e))
        return {'msg' : str(e)}

@app.get("/save_map_slam/{name}") 
def save_map_slam(name: Union(str, None), response: Response):
    global ros_interface
    try:
        if ros_interface is None:
            return JSONResponse(status_code=400, content={'msg' : "Ros interface not launched"}) 

        dest = f"/data/{name}" if name is not None else "/data/map"

        ret = ros_interface.send_request_slam_save_map(destination=dest)
        if ret is None : 
            return JSONResponse(status_code=400, content={'msg' : "Ros service unreachable"}) 
        elif not ret.success :
            return JSONResponse(status_code=400, content={'msg' : "Can't save map : " + ret.message}) 
        else : 
            return {'msg' : "Map saved"}

    except Exception as e:
        print(str(e))
        return {'msg' : str(e)}

@app.get("/potree/{name}")
def potree(name: Union(str, None), response: Response):
    global ros_interface
    try:

        if ros_interface is None:
            return JSONResponse(status_code=400, content={'msg' : "Ros interface not launched"}) 

        if not ros_interface.map_saved:
            return JSONResponse(status_code=200, content={'msg' : 'Please save the map before transforming it'}) 

        dest = f"/data/{name}" if name is not None else "/data/map"

        ret = pipeplineExportWithColors(dest)
        print(ret)
        if not (ret['status'] == 'ok') :
            return JSONResponse(status_code=400, content={'msg' : ret['msg']}) 
        
        ros_interface.set_potree_path(ret['msg']['potreePath']) 
        
        return {"msg": ret['msg'] }

    except Exception as e : 
        print(str(e))
        return JSONResponse(status_code=400, content={'msg' : str(e)}) 

@app.get("/zipPotree")
def potree(response: Response):
    global ros_interface
    try:

        if ros_interface is None:
            return JSONResponse(status_code=400, content={'msg' : "Ros interface not launched"}) 

        if ros_interface.get_potree_path() == None : 
            return JSONResponse(status_code=200, content={'msg' : 'Please export to potree first'}) 

        ret = createZip(ros_interface.get_potree_path()) 
        print(ret)
        if not (ret['status'] == 'ok') :
            return JSONResponse(status_code=400, content={'msg' : ret['msg']}) 
        return {"msg": ret['msg'] }

    except Exception as e : 
        print(str(e))
        return JSONResponse(status_code=400, content={'msg' : str(e)}) 

# @app.get("/send2Site3d")
# def send2Site3d(response: Response, realtityCaptureName : str = None):
#     global ros_interface
#     try:
#         if ros_interface is None:
#             return JSONResponse(status_code=400, content={'msg' : "Ros interface not launched"}) 

#         if ros_interface.get_potree_path() == None : 
#             return JSONResponse(status_code=200, content={'msg' : 'Please export to potree first'}) 

#         with RestClientPE(base_url=TB_url) as rest_client:
#             try:
#                 # Auth with credentials
#                 rest_client.login(username=TB_user, password=TB_password)
#                 current_user = rest_client.get_user()
#                 token = rest_client.get_user_token(current_user.id)
#                 token = token.token

#                 retZip =  createZip(ros_interface.get_potree_path()) 
#                 print(retZip)
#                 RCName = realtityCaptureName
#                 jobsiteId = '339446f0-6c37-11ed-8aba-99170584c638'
   
#                 retSite3d = sendPointCloud2Cloud(ros_interface.get_potree_path(),url = SITE3D_url, RCName=RCName, jobsiteId=jobsiteId,  token = token)
#                 print(retSite3d)
#                 if not (retSite3d['status'] == 'ok') : 
#                     return JSONResponse(status_code=400, content={'msg' : retSite3d['msg']}) 
                
#                 return {'msg' : {"msgSite3d": retSite3d['msg'] , 'msgZip' : retZip['msg']}}

#             except Exception as e : 
#                 print(str(e))
#                 return JSONResponse(status_code=400, content={'msg' : str(e)}) 


#     except Exception as e : 
#         print(str(e))
#         return JSONResponse(status_code=400, content={'msg' : str(e)}) 



@app.get("/rosTimeStamp")
def rosTimeStamp(response: Response):
    global ros_interface
    try:
        if ros_interface is None:
            return JSONResponse(status_code=400, content={'msg' : "Ros interface not launched"}) 
        
        ts = ros_interface.get_timestamp()
        
        return {'msg' : {'ts' : {'sec' : ts.sec, 'nanosec' : ts.nanosec}}}



    except Exception as e : 
        print(str(e))
        return JSONResponse(status_code=400, content={'msg' : str(e)})         



# @app.get("/send2Site3d")
# def zipPotree(response: Response, realtityCaptureName : str = None):
#     try : 
        

#         with RestClientPE(base_url=TB_url) as rest_client:
#             try:
#                 # Auth with credentials
#                 rest_client.login(username=TB_user, password=TB_password)
#                 current_user = rest_client.get_user()
#                 token = rest_client.get_user_token(current_user.id)
#                 retZip = createZip(scan.get_potreePath()) 
#                 if not (retZip['status'] == 'ok') : 
#                     return JSONResponse(status_code=400, content={'msg' : retZip['msg']}) 
                
#                 RCName = realtityCaptureName
#                 jobsiteId = '339446f0-6c37-11ed-8aba-99170584c638'
                
#                 token = token.token
#                 retSite3d = sendPointCloud2Cloud(scan.get_potreePath(),url = SITE3D_url, RCName=RCName, jobsiteId=jobsiteId,  token = token)
#                 if not (retSite3d['status'] == 'ok') : 
#                     return JSONResponse(status_code=400, content={'msg' : retSite3d['msg']}) 
                
#                 return {'msg' : {"msgSite3d": retSite3d['msg'] , 'msgZip' : retZip['msgZip']}}
#             except Exception as e : 
#                 print(e) 
#                 return JSONResponse(status_code=400, content={'msg' : str(e)}) 
#     except Exception as e :
#         print(e)
#         return JSONResponse(status_code=400, content={'msg' : str(e)}) 
