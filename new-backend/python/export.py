import sys
import open3d as o3d
import random
import time
import laspy 
import subprocess
import sys
import os
from datetime import datetime
import numpy as np

def exportToPotree(pcd):
    try:
        now = datetime.now() #
        
        tagTime = now.strftime("%Y%m%d%H%M%S")

        allPoints = np.array(pcd.points)
    
        if not pcd.has_colors():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=40), 
                fast_normal_computation=False)
        
            pcd.normalize_normals()
            
            c = np.abs(np.asarray(pcd.normals))
            pcd.colors = o3d.utility.Vector3dVector(o3d.utility.Vector3dVector((c+1)*0.5))
            
        colors = np.array(pcd.colors)*254
    
        # 1. Create a new header
        print("--------------------")
        print("Post-processing...")
        print("Exporting to Potree... ")
        print("--------------------")
        pf = laspy.point.format.PointFormat(3)
        header = laspy.LasHeader(point_format=pf, version="1.4")
        
        #header.offsets =np.min(pc_clean, axis=0)
        #header.scales = np.array([1,1,1])
        new_las = laspy.LasData(header)
        new_las.points.resize(allPoints.shape[0])
        
        
        
        new_las.x = allPoints[:,0]
        new_las.y = allPoints[:,1]
        new_las.z = allPoints[:,2]
            
        
        new_las.red   = colors[:,0]
        new_las.green = colors[:,1]
        new_las.blue  = colors[:,2]
        
            
        new_las.header.min = np.min(allPoints, axis=0) -1 
        new_las.header.max = np.max(allPoints, axis=0) +1
        PotreePath = '/data/spot' + tagTime
        LasFile = PotreePath + '/map_go.las'

        if not os.path.exists(PotreePath):
            os.makedirs(PotreePath)

        new_las.write(LasFile)
        print("Export Laz Done")

        PotreeConverterPath = '/src/PotreeConverter-master/build/PotreeConverter'
        
        pageName = 'spot'
        result = subprocess.run(
                [PotreeConverterPath, '-i',   LasFile  , '-o', PotreePath,  '--generate-page' , pageName],
                capture_output=True,  
                text=True
        )
        print(result)
        return {'status' : 'ok', 'msg' : {'potreeResult' : result, 'potreePath' :PotreePath}}
    except Exception as e:
        print(e)
        return {'status' : 'ko' , 'msg' : str(e)}


def createZip(potreePath):
    import zipfile
    import os
    try :
        os.chdir(potreePath+'/pointclouds/spot/')
        filenames = ["octree.bin", "metadata.json", "hierarchy.bin"]
        with zipfile.ZipFile("pointclouds.zip", mode="w") as archive:
            for filename in filenames:
                archive.write(filename)
        return {'status' : 'ok', 'msg' : 'zip compression completed'}
    except Exception as e:
        print(e)
        return {'status' : 'ko' , 'msg' : str(e)}
def sendPointCloud2Cloud(potreePath, 
    jobsiteId,  
    token,
    url = None,
    RCName=None, date=None):
    import requests
    import os
    import datetime
    try:
        os.chdir(potreePath+'/pointclouds/spot/')

        headers = {}
        headers['accept'] = '*/*'

        api_link =  "/api/reality-captures"
        headers['Authorization'] = 'bearer ' + token

        file = {'realityCapture': ('pointclouds.zip', open('pointclouds.zip', 'rb'))}
        now = datetime.datetime.now()
        if date == None :
            date = now.strftime("%Y-%m-%dT%H:%M:%S.000Z")
        if (RCName == None) : 
            RCName = 'SpotRealityCapture_%s' % now.strftime("%Y-%m-%d %H:%M:%S")
        
        data = {}
        data["name"] = RCName
        data["date"] = date 
        data["jobsiteId"] = jobsiteId
        r = requests.post(url + api_link, files=file, headers = headers, data=data)
        return {'status': 'ok', 'msg' : r.text}
    except Exception as e:
        print(e)
        return {'status' : 'ko' , 'msg' : str(e)}
    

def pipeplineExportWithColors(folderName):
    try : 
        baseName = f"/data/{folderName}/map" 
        fileFormat = "pcd"
        outFilename="%s.%s" % (baseName, fileFormat)
        pcd = o3d.io.read_point_cloud(outFilename)
        msg = exportToPotree(pcd)
        if msg['status'] == 'ok' :   return msg
        else : return {'status' : 'ko', 'msg' : msg['msg']}
    except Exception as e:
        print(e)
        return {'status' : 'ko' , 'msg' : str(e) }

