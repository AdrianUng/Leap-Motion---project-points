from scipy import io as sio
import xml.etree.ElementTree as ET
import numpy as np

###############################################################################
def load_camera_param(folder):
    camera_params = {}

    focal_length = sio.loadmat(folder + '\\' + 'FocalLength.mat')
    camera_params['focalLength'] = focal_length['ff']

    intrinsicMat = sio.loadmat(folder + '//' + 'IntrinsicMatrix.mat')
    camera_params['intrinsicMatrix'] = intrinsicMat['ff']

    return camera_params
###############################################################################
def export_as_XML(file_name,finger_joints_xy,output_folder): #Leap Coordinates!
    new_root = ET.Element("leap_coordinates")

    img_name=ET.SubElement(new_root,"img_name")
    img_name.text=file_name

    nr_points=np.size(finger_joints_xy,1)
    finger_joints=ET.SubElement(new_root,"finger_joints", ptCount=str(nr_points))
#    if np.size(finger_joints,)
    for i in range(0,nr_points):
        point_nr="joint%d" % (i+1)
        ET.SubElement(finger_joints,point_nr, x=str(finger_joints_xy[0][i]), y=str(finger_joints_xy[1][i]), z=str(finger_joints_xy[2][i]))
    tree = ET.ElementTree(new_root)
    tree.write(output_folder+file_name+'.xml')
    # wrap it in an ElementTree instance, and save as XML
    print('XMLs created')
    return None
###############################################################################
def camera_coord_export_as_XML(file_name,finger_joints_xy,output_folder): # Camera Coordinates!
    new_root = ET.Element("leap_coordinates")

    img_name=ET.SubElement(new_root,"img_name")
    img_name.text=file_name
    finger_joints_xy=np.floor(finger_joints_xy)
    nr_points=np.size(finger_joints_xy,1)
    finger_joints=ET.SubElement(new_root,"finger_joints", ptCount=str(nr_points))

    for i in range(0,nr_points):
        point_nr="joint%d" % (i+1)
        ET.SubElement(finger_joints,point_nr, x=str(finger_joints_xy[0,i]), y=str(finger_joints_xy[1,i]))
    tree = ET.ElementTree(new_root)
    tree.write(output_folder+file_name+'.xml')
    # wrap it in an ElementTree instance, and save as XML
    print('XMLs created')
    return None
###############################################################################