import numpy as np
import cv2
import glob
#import matplotlib.pyplot as plt
from math import ceil
import pickle
#%matplotlib qt

def main():
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    x = 9
    y = 6
    objp = np.zeros((y*x,3), np.float32)
    objp[:,:2] = np.mgrid[0:x,0:y].T.reshape(-1,2)
    dist_mtx_file = 'undist.p'

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d points in real world space
    imgpoints = [] # 2d points in image plane.

    # Make a list of calibration images
    #images = glob.glob(os.path.join(data_folder, 'camera_cal/calibration*.jpg'))
    images = glob.glob('./camera_cal/calibration2.jpg')

    #rows = ceil(len(images)/2.0)
    rows = len(images)
    #plt.figure(figsize=(20,60))
    # Step through the list and search for chessboard corners
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (x,y),None)

        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)
        else:
            print(f"Chessboard not found, file: {fname}")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    if ret:
        p_number = 0
        while True:
            fname = images[p_number]
            img = cv2.imread(fname)

            # Draw and display the corners
            #img = cv2.drawChessboardCorners(img, (9,6), corners, ret)
            dst = cv2.undistort(img, mtx, dist, None, mtx)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img, fname, (10,30), font, 1, (255,255,0), 2, cv2.LINE_AA)
            cv2.imshow('source',  img) #np.hstack((img, dst)))
            cv2.putText(dst, fname, (10,30), font, 1, (255,255,0), 2, cv2.LINE_AA)
            cv2.imshow('undistorted',  dst) #np.hstack((img, dst)))
            
            k = cv2.waitKey(10)
            # Press q to break
            if k == ord('q'):
                break
            # press a to increase p_number by 1
            if k == ord('a'):
                p_number +=1
                if p_number >=len(images)-1:
                    p_number = len(images)-1
            # press d to decrease p_number by 1
            elif k== ord('d'):
                p_number -= 1
                if p_number <=0:
                    p_number = 0

    dist_pickle = {}
    dist_pickle['mtx'] = mtx
    dist_pickle['dist'] = dist
    with open(dist_mtx_file, 'wb') as config_dictionary_file:
        pickle.dump(dist_pickle, config_dictionary_file)
        print('distortion matrix saved to {}'.format(dist_mtx_file))

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
