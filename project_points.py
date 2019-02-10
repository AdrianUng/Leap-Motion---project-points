"""
Projecting the LEap Coordinates onto the Webcam image feed

1) The Rotation matrix = Identity Matrix, translation vector = Zeros
2) There is a slight misalignment between the Leap Motion and Webcam that is taken into consideration (40mm on the Oy axis)
3) Export (Leap 3D, Camera 2D) Coordinates into XML files
4) Save the corresponding RGB images
author: Adrian Ungureanu (https://github.com/AdrianUng)
license: MIT
"""
import sys, cv2, os, numpy as np
sys.path.insert(0, 'lib/') # path to Python 2.7 libraries for Leap Motion API
import Leap # importing Leap object

from my_functions import export_as_XML, load_camera_param, camera_coord_export_as_XML

# Extract the xyz coordinates of the joint
def joint_coord(finger, bone_type):
    # Defining what (finger) joints are saved in variables
    if bone_type is 'meta':
        coordinates = finger.bone(Leap.Bone.TYPE_METACARPAL).next_joint
    elif bone_type is 'proximal':
        coordinates = finger.bone(Leap.Bone.TYPE_PROXIMAL).next_joint
    elif bone_type is 'intermediate':
        coordinates = finger.bone(Leap.Bone.TYPE_INTERMEDIATE).next_joint
    elif bone_type is 'distal':
        coordinates = finger.bone(Leap.Bone.TYPE_DISTAL).next_joint

    x_meta = round(coordinates[0], 0)
    y_meta = round(coordinates[1], 0)
    z_meta = round(coordinates[2], 0)
    meta = Leap.Vector(x_meta, y_meta, z_meta)

    return meta
##################################################################
def finger_coordinates(fingers, bone_type):
    # Extract the xyz coordinates for every joint in the finger ...
    coord = []
    for fing in fingers:
        coord.append(joint_coord(fing, bone_type))
    return coord
##################################################################
def project_3DPoints(xyz_all):
    # recover Homography matrix
    global H
    # format X,Y,Z coordinates
    xyz_all = np.concatenate((np.array(xyz_all), np.ones((1, 20))), axis=0)
    xyz_all = np.transpose(xyz_all)

    # recover offset (based on webcam-leap motion physical displacement)
    global offset
    xyz_all[:, 0] = xyz_all[:, 0] * -1 + offset[0]  # Leap Ox axis is reversed compared to Camera Ox
    # insert Oy and Oz offset corresponding to the 2 coordinate systems!
    xyz_all[:, 1] = xyz_all[:, 1] + offset[1]
    xyz_all[:, 2] = xyz_all[:, 2] + offset[2]

    # Determine corresponding 2D coordinates
    new2D_coord = np.dot(H, np.transpose(xyz_all))
    scale = np.array([np.transpose(new2D_coord[2])])
    new2D_coord_final = np.divide(new2D_coord[:2, :], scale)
    ratio_x = 640.0 / np.float(xlim_im)
    ratio_y = 480.0 / np.float(ylim_im)
    new2D_coord_final[0, :] = np.floor(np.divide(new2D_coord_final[0, :], ratio_x))
    new2D_coord_final[1, :] = np.floor(np.divide(new2D_coord_final[1, :], ratio_y))

    return new2D_coord_final

##################################################################
##################################################################
##################################################################
def run(controller):
    cam = cv2.VideoCapture(0)   # Initializing camera object
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, xlim_im)  # (x,y) size of captured images
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, ylim_im)

    ################################################################################
    # # Camera parameters - Intrnsic matrix, Extrinsic matrix, Homography matrix # #
    ################################################################################
    # Computed previously, loaded here...
    camera_params = load_camera_param('cameraParams')
    global intr_mat
    intr_mat = camera_params['intrinsicMatrix']
    intr_mat[0][2] = intr_mat[2][0]
    intr_mat[1][2] = intr_mat[2][1]
    intr_mat[2][0] = 0
    intr_mat[2][1] = 0

    R = np.identity(3) # Rotation matrix
    t = np.zeros((3, 1)) # Translation vector

    Rt = np.zeros((3, 4))
    K = np.zeros((4, 3))

    Rt[0:3, 0:3] = R
    Rt[0:4, [3]] = t

    K[:3, :3] = intr_mat # intrinsic  Matrix 4x4
    global H
    H = np.dot(K, Rt) # Homography matrix
    #########################################################################
    # #           Acquiring and projecting coordinates of joints          # #
    #########################################################################

    font = cv2.FONT_HERSHEY_SIMPLEX # used when visualizing points...

    while (True):
        frame = controller.frame()  # Load the frame (Leap Motion API)
        hands = frame.hands  # extract the hand object from the frame

        if hands.is_empty == False:  # if the extracted hand is not empty
            my_hand = hands[0]  # Save the first hand
            my_arm = my_hand.arm  # extract the Arm object from the Hand object

            # Extract the wrist xyz coordinates from the Arm object (not used here)
            my_wrist = my_arm.wrist_position
            # Extract the palm center xyz coordinates from the Hand object (not used here)
            hand_center = my_hand.stabilized_palm_position
            # Load the hand confidence from the Hand object
            hand_confidence = my_hand.confidence

            # Dictionary containing the joint coordinates
            bone_set = ['meta', 'proximal', 'intermediate', 'distal'] # the 4 finger joints
            joint_coord = {bone_set[0]: finger_coordinates(my_hand.fingers, bone_set[0]),
                           bone_set[1]: finger_coordinates(my_hand.fingers, bone_set[1]),
                           bone_set[2]: finger_coordinates(my_hand.fingers, bone_set[2]),
                           bone_set[3]: finger_coordinates(my_hand.fingers, bone_set[3])}

            # Define lists that will contain the xy coordinates
            xs_all = [] # original x coordinates
            ys_all = [] # y coord
            zs_all = [] # z coord
            for i in range(0, 5):
                # Load the xz (Leap coord) coordinates into a list to plot them
                xs = [joint_coord[bone_set[0]][i][0], joint_coord[bone_set[1]][i][0],
                      joint_coord[bone_set[2]][i][0], joint_coord[bone_set[3]][i][0]]
                xs_all = np.concatenate((xs_all, xs), axis=0)

                ys = [joint_coord[bone_set[0]][i][2], joint_coord[bone_set[1]][i][2],
                      joint_coord[bone_set[2]][i][2], joint_coord[bone_set[3]][i][2]]
                ys_all = np.concatenate((ys_all, ys), axis=0)

                zs = [joint_coord[bone_set[0]][i][1], joint_coord[bone_set[1]][i][1],
                      joint_coord[bone_set[2]][i][1], joint_coord[bone_set[3]][i][1]]
                zs_all = np.concatenate((zs_all, zs), axis=0)

            xyz_all = [xs_all, ys_all, zs_all]

            xy_projected = project_3DPoints(xyz_all) #
            xy_projected = np.transpose(np.array(xy_projected, dtype=np.int32))

            ######################################################
            # #           Reading frames from Webcam           # #
            ######################################################
            # #           Plotting projected points            # #
            ######################################################
            ret, image = cam.read()
            h_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            h_image = cv2.resize(h_image,(xlim_im/2,ylim_im/2))

            cv2.polylines(h_image, [xy_projected/2], False, (0, 0, 255),thickness=2)
            cv2.putText(h_image, 'idNr%d_inst_%d_image_nr%d' % (id_nr,instance,global_nr),
                        (200, 25), font, 1, (135, 0, 0), 2)#, cv2.LINE_AA)

            for mm in np.arange(0,20,1):
                cv2.circle(h_image, (xy_projected[mm, 0] / 2, xy_projected[mm, 1] / 2), 5, (255, 0, 0), -1)
                if mm in [0,1,4,8,12,16]:
                    cv2.circle(h_image,(xy_projected[mm,0]/2,xy_projected[mm,1]/2),5,(0,255,0),-1)
            cv2.imshow('webcam image', cv2.cvtColor(h_image, cv2.COLOR_BGR2RGB))

            ####################################################
            # #           Saving image + XML files           # #
            ######################################################
            # #           Press 's':    Save image/coordinates # #
            # #           Press 'q':    Stop code/ Exit        # #
            ######################################################
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): #
                sys.exit(0)
            elif key == ord('s'):
                global global_nr
                file_name = 'idNr%d_inst_%d_image_nr%d' % (id_nr, instance, global_nr)
                file_name_leapC = file_name + '_leapCoord'
                file_name_camC = file_name + '_camCoord'

                print(file_name)

                global_nr = global_nr + 1   # saved image counter
                plm_name = xml_output + file_name + '.png' # saved image name
                saved_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # saved image
                cv2.imwrite(plm_name, cv2.cvtColor(saved_image, cv2.COLOR_BGR2RGB)) # writing saved image to destination

                export_as_XML(file_name_leapC, xyz_all, xml_output) # export Leap coordinates as XML file to destination
                camera_coord_export_as_XML(file_name_camC, xy_projected.transpose(), xml_output) # export camera coordinates
                # as XML file to destination

                fig_name = xml_output + file_name + '_overlapped.jpg' # overlapped image (from figure)
                cv2.imwrite(fig_name,cv2.cvtColor(h_image, cv2.COLOR_BGR2RGB)) # writing overlapped image to destination
##################################################################
##################################################################
##################################################################


def main():
    controller = Leap.Controller()  # instatiating a Leap Controller object
    controller.set_policy_flags(Leap.Controller.POLICY_IMAGES)

    try:
        run(controller)

    except KeyboardInterrupt:
        sys.exit(0)

###############################################################################
# # #                          Global Constraints                         # # #
###############################################################################
# Rows (y), Columns (x) for saved webcam images
xlim_im = 1600 #640  # 160#640
ylim_im = 1200 #480  # 90#480
###############################################################################
offset = [5, -40, -10]
session_nr = 000    # session nr
id_nr = 0   # id corresponding to participant
instance = 0    # instance nr
folder_name = 'id%d_%d/' % (id_nr, instance)    # destination folder name
###############################################################################
# # #                Checking if the folder already exists...             # # #
###############################################################################
xml_output = 'session%d/%s' % (session_nr, folder_name)
if not os.path.exists(xml_output):
    print('directory %s created' % xml_output)
    os.makedirs(xml_output)
else:
    print('directory already exists')

global_nr = 0 # image counter (global variable)

if __name__ == '__main__':
    main()