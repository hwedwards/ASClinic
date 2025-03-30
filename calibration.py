import numpy as np
import cv2
import glob

NUM_CB_ROWS = 9
NUM_CB_COLS = 6
CB_SQUARE_SIDE_LENGTH = 0.0232 # in millimeters (confirm in documentation)

# Specify the termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare the "world frame" coordinates of the chessboard corners,
# referred to as the "object points",
# i.e., coordinates of the form (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0),
# noting that the third coordinate is always zero by convention
objp = np.zeros((NUM_CB_ROWS*NUM_CB_COLS,3), np.float32)
objp[:,:2] = np.mgrid[0:NUM_CB_ROWS,0:NUM_CB_COLS].T.reshape(-1,2)

# Scale by the side length of a single the chessboard square
objp = objp * CB_SQUARE_SIDE_LENGTH

# Print out the object points for a visual check:
print("World frame coordinates to be used for the chessboard corners:")
print("objp =")
print(str(objp))

# Initialze arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Get all the jpg images in the current directory
images = glob.glob('*.jpg')

# Iterate over all the images found
for fname in images:
    print("\nNow processing image: " + fname)
    # Read in the image
    img = cv2.imread(fname)
    # Convert the image to gray scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (NUM_CB_ROWS,NUM_CB_COLS), None)
    # If found
    if ret == True:
        print("> Chessboard corners FOUND")
        # > Add object points
        objpoints.append(objp)
        # > Refine the image points and then add them
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # > Draw the located corners onto the image
        cv2.drawChessboardCorners(img, (NUM_CB_ROWS,NUM_CB_COLS), corners2, ret)
        # > Save the image to file for later checking
        temp_filename = fname + "_with_corners.jpg"
        cv2.imwrite(temp_filename,img)
        print("> Saved image to: " + temp_filename)
        # > Display the image for immediate checking
        #cv2.imshow('img', img)
        # > Wait breifly before moving on to the next iteration
        cv2.waitKey(500)
    else:
        print("> Chessboard corners NOT found")

print("\nNow computing the camera calibration.")
# Call the function to calibrate the camera, this returns
# > mtx and dist, which are the intrinsic camera parameters
# > rvecs and tvecs, which are the extrinsic parameters for each image
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Display the intrinsic camera parameters
print("> Camera calibration complete.")
print("> The intrinsic camera parameter estimates are:")
print("mtx  = ")
print(str(mtx))
print("dist = " + str(dist))

cv2.destroyAllWindows()