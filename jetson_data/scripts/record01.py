import cv2
import os

print(cv2.__version__)

## Create Camera Object
# Camera Display Settings
dispW = 420
dispH = 420
flip  = 2
# Camera Capture Settings
capW = 420
capH = 420
fps = 30

# Camera G-Streamer Settings
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width='+str(capW)+', height='+str(capH)+', format=NV12, framerate='+str(fps)+'/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

#camSet ='v4l2src device=/dev/video1 ! video/x-raw,width='+str(width)+',height='+str(height)+'   ,framerate=24/1 ! videoconvert ! appsink'
video =cv2.VideoCapture(camSet)


#video = cv2.VideoCapture(0)

#s = s1 + '_' + format(i, '05') + '_' + s2 + '_' + format(f, '.5f')

counter_x = 0
#n_avi = 'output'
exten_file = '.avi'
#file_avi = '{}{}{}'.format(counter_x, n_avi, exten_file)

filename = "file{}.avi"
while os.path.isfile(filename.format(counter_x)):
    counter_x += 1
filename = filename.format(counter_x)

# if os.path.isfile(filename):
#     print('file_exists')
#     print(filename)
#     print(counter_x)
# else:
#     print('not_exist')

# We need to check if camera
# is opened previously or not
if (video.isOpened() == False):
    print("Error reading video file")

# We need to set resolutions.
# so, convert them from float to integer.
frame_width = int(video.get(3))
frame_height = int(video.get(4))


size = (frame_width, frame_height)
 # check file is exit



# Below VideoWriter object will create
# a frame of above defined The output
# is stored in 'output.avi' file.
#cv2.VideoWriter(filename, fourcc, fps, frameSize)
result = cv2.VideoWriter(filename ,
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         30, size)


while(True):


    ret, frame = video.read()


    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if ret == True:

            # Write the frame into the
            # file 'output.avi'
        result.write(frame)
            
            # Display the frame
            # saved in the file
        cv2.imshow('frame', frame)



    if cv2.waitKey(1) & 0xFF == ord('q'):
        break



video.release()
result.release()
cv2.destroyAllWindows()
print("The video was successfully saved")
