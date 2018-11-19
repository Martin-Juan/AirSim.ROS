import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)
running = True

while running is True :
        command = str(input('what do you want to do move(1) , getimage(2) or quit(3)'))
        if command == 1 :
                x = input('chose x coordinate')
                y = input ('chose y coordinate')
                z = input ('chose z coordinate')
                speed = input('chose speed in m/s')

                client.moveToPositionAsync(x, y, z, speed).join()

                client.hoverAsync().join()

                state = client.getMultirotorState()
                print("state: %s" % pprint.pformat(state))
        elif command == 2 :
                responses = client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
                    airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True), #depth in perspective projection
                    airsim.ImageRequest("1", airsim.ImageType.Scene), #scene vision image in png format
                    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
        elif command == 3:
                running = False
        


tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

for idx, response in enumerate(responses):

    filename = os.path.join(tmp_dir, str(idx))

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) #get numpy array
        img_rgba = img1d.reshape(response.height, response.width, 4) #reshape array to 4 channel image array H X W X 4
        img_rgba = np.flipud(img_rgba) #original image is flipped vertically
        img_rgba[:,:,1:2] = 100 #just for fun add little bit of green in all pixels
        airsim.write_png(os.path.normpath(filename + '.greener.png'), img_rgba) #write to png

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
