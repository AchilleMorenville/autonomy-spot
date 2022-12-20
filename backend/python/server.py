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

from tb_rest_client.rest_client_pe import Device, RestClientPE
from tb_rest_client.rest import ApiException

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

@app.get("/save_map_slam") 
def save_map_slam(response: Response):
    global ros_interface
    try:
        if ros_interface is None:
            return JSONResponse(status_code=400, content={'msg' : "Ros interface not launched"}) 

        ret = ros_interface.send_request_slam_save_map(destination="/data")
        if ret is None : 
            return JSONResponse(status_code=400, content={'msg' : "Ros service unreachable"}) 
        elif not ret.success :
            return JSONResponse(status_code=400, content={'msg' : "Can't reset slam : " + ret.message}) 
        else : 
            return {'msg' : "Slam reset"}

    except Exception as e:
        print(str(e))
        return {'msg' : str(e)}

@app.get("/potree")
def potree(response: Response):
    global ros_interface
    try:

        if ros_interface is None:
            return JSONResponse(status_code=400, content={'msg' : "Ros interface not launched"}) 

        if not ros_interface.map_saved:
            return JSONResponse(status_code=200, content={'msg' : 'Please save the map before transforming it'}) 

        ret = pipeplineExportWithColors()
        print(ret)
        if not (ret['status'] == 'ok') :
            return JSONResponse(status_code=400, content={'msg' : ret['msg']}) 
        
        ros_interface.set_potree_path(ret['msg']['potreePath']) 
        
        return {"msg": ret['msg'] }

    except Exception as e : 
        print(str(e))
        return JSONResponse(status_code=400, content={'msg' : str(e)}) 


# @app.get("/reset")
# def reset(response: Response):
#     try : 
#         scan = capture_slam_data.scanning()
#         return {"msg": "reset done"}
    
#     except Exception as e : 
#         print(str(e))
#         return {'msg' : str(e)}

# @app.get("/connect2Spot", status_code=200)
# def connect2spot(response: Response):
#     try :
#         user = os.getenv("SPOT_USER")
#         password = os.getenv("SPOT_PWD")
#         spot_ip = os.getenv("SPOT_IP")
#         ret = scan.connect(spot_ip = spot_ip, user=user, password=password)
        
#         if not (ret['status'] == 'ok') : 
#             return JSONResponse(status_code=400, content={'msg' : ret['msg']}) 
#         else  : return {'msg' : ret['msg']}
    
#     except Exception as e : 
#         print(str(e))
#         return JSONResponse(status_code=400, content={'msg' : str(e)}) 


# @app.get("/stop_slam")
# async def stop_slam(response: Response, background_tasks: BackgroundTasks):
#     try : 
#         background_tasks.add_task(scan.stop_scan)
#         return {"msg":"stop command sent..."}
#     except Exception as e : 
#         print(str(e))
#         return JSONResponse(status_code=400, content={'msg' : str(e)}) 

# @app.get("/start_slam")
# async def start_slam(response: Response, background_tasks: BackgroundTasks, timeSleep : float =0.2):
#     try:
#         if scan.is_connected : 
#             background_tasks.add_task(scan.start_scan, timeSleep)
#             return {"msg":"SLAM Algo started", 'sleep' : timeSleep}
#         else :
#             return {"msg":"SLAM Algo NOT started\n please first connect to SPOT"} 
#     except Exception as e : 
#         print(str(e))
#         return JSONResponse(status_code=400, content={'msg' : str(e)}) 

# @app.get("/status_slam")
# def slam_status(response: Response):
#     try:
#         ret = scan.get_scanning_status()
#         if not (ret['status'] == 'ok') : 
#             return JSONResponse(status_code=400, content={'msg' : ret['msg']}) 
#         return {'msg' : ret['msg']}

#     except Exception as e : 
#         print(str(e))
#         return JSONResponse(status_code=400, content={'msg' : str(e)}) 


# @app.get("/potree")
# def potree(response: Response):
#     try:
#         if not os.path.exists('/data/saved_file_finished'):
#             return JSONResponse(status_code=200, content={'msg' : 'Please wait until save process finished'}) 
#         ret = pipeplineExportWithColors()
#         print(ret)
#         if not (ret['status'] == 'ok') :
#             return JSONResponse(status_code=400, content={'msg' : ret['msg']}) 
#         scan.set_potreePath(ret['msg']['potreePath']) 
#         return {"msg": ret['msg'] }

#     except Exception as e : 
#         print(str(e))
#         return JSONResponse(status_code=400, content={'msg' : str(e)}) 
    
# @app.get("/zipPotree")
# def zipPotree(response: Response):
#     try:
#         ret = createZip(scan.get_potreePath()) 
#         if not (ret['status'] == 'ok') : 
#             return JSONResponse(status_code=400, content={'msg' : ret['msg']}) 
    
#         else : return {'msg' : ret['msg']}

#     except Exception as e : 
#         print(str(e))
#         return JSONResponse(status_code=400, content={'msg' : str(e)}) 


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