import os
import time
from absl import app, logging
try:
   import cv2
except ImportError:
   try:
      os.rename('/opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so',
                '/opt/ros/kinetic/lib/python2.7/dist-packages/_cv2.so')
      import cv2
      os.rename('/opt/ros/kinetic/lib/python2.7/dist-packages/_cv2.so',
                '/opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so')
   except ImportError:
      os.rename('/opt/ros/kinetic/lib/python2.7/dist-packages/_cv2.so',
                '/opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so')
   #except FileNotFoundError:
import numpy as np
import tensorflow as tf
from yolov3_tf2.models import (
    YoloV3, YoloV3Tiny
)
from yolov3_tf2.dataset import transform_images, load_tfrecord_dataset
from yolov3_tf2.utils import draw_outputs
from flask import Flask, request, Response, jsonify, send_from_directory, abort

# customize your API through the following parameters
classes_path = './data/labels/obj.names'
weights_path = './weights/yolov3_c_last.tf'
tiny = False                    # set to True if using a Yolov3 Tiny model
size = 416                      # size images are resized to for model
output_path = './detections/'   # path to output folder where images with detections are saved
num_classes = 7                # number of classes in model

# load in weights and classes
physical_devices = tf.config.experimental.list_physical_devices('GPU')
if len(physical_devices) > 0:
    tf.config.experimental.set_memory_growth(physical_devices[0], True)

if tiny:
    yolo = YoloV3Tiny(classes=num_classes)
else:
    yolo = YoloV3(classes=num_classes)

yolo.load_weights(weights_path).expect_partial()
print('weights loaded')

class_names = [c.strip() for c in open(classes_path).readlines()]
print('classes loaded')

# Initialize Flask application
app = Flask(__name__)

import io
#from PIL import Image
#im = Image.open('test.jpg')
#im.save(buf, format='JPEG')
#byte_im = buf.getvalue()

# API that returns JSON with classes found in images
@app.route('/detections', methods=['POST'])
def get_detections():
    whole_t1 = time.time()
    raw_images = []
    images = request.files.getlist("images")
    image_names = []
    for image in images:
        image_name = image.filename
        image_names.append(image_name)
        #image.save(os.path.join(os.getcwd(), image_name))
        buf = io.BytesIO()
        image.save(buf)
        #print(type(image))
        img_raw = tf.image.decode_image(
            #open(image_name, 'rb').read(), channels=3)
            buf.getvalue(), channels=3)
        raw_images.append(img_raw)
        
    num = 0
    
    # create list for final response
    response = []

    for j in range(len(raw_images)):
        # create list of responses for current image
        responses = []
        raw_img = raw_images[j]
        num+=1
        #print('raw_img size: ', raw_img)
        img = tf.expand_dims(raw_img, 0)
        img = transform_images(img, size)

        t1 = time.time()
        boxes, scores, classes, nums = yolo(img)
        t2 = time.time()
        print('time: {}'.format(t2 - t1))

        print('detections:')
        for i in range(nums[0]):
            print('\t{}, {}, {}'.format(class_names[int(classes[0][i])],
                                            np.array(scores[0][i]),
                                            np.array(boxes[0][i])))
            responses.append({
                "class_id": int(classes[0][i]),
                "class_name": class_names[int(classes[0][i])],
                "confidence": float("{0:.2f}".format(np.array(scores[0][i])*100))
            })
        response.append({
            "image": image_names[j],
            "detections": responses
        })
        #img = cv2.cvtColor(raw_img.numpy(), cv2.COLOR_RGB2BGR)
        #img = draw_outputs(img, (boxes, scores, classes, nums), class_names)
        #cv2.imwrite(output_path + 'detection' + str(num) + '.jpg', img)
        #print('output saved to: {}'.format(output_path + 'detection' + str(num) + '.jpg'))

    #remove temporary images
    #for name in image_names:
    #    try:
    #        os.remove(name)
    #    except FileNotFoundError:
    #        print('temp file not found: ', name)
    whole_t2 = time.time()
    #print('whole time: {}'.format(whole_t2 - whole_t1))
    try:
        return jsonify({"response":response}), 200
    except FileNotFoundError:
        abort(404)

# API that returns image with detections on it
@app.route('/image', methods= ['POST'])
def get_image():
    image = request.files["images"]
    image_name = image.filename
    image.save(os.path.join(os.getcwd(), image_name))
    img_raw = tf.image.decode_image(
        open(image_name, 'rb').read(), channels=3)
    img = tf.expand_dims(img_raw, 0)
    img = transform_images(img, size)

    t1 = time.time()
    boxes, scores, classes, nums = yolo(img)
    t2 = time.time()
    print('time: {}'.format(t2 - t1))

    print('detections:')
    for i in range(nums[0]):
        print('\t{}, {}, {}'.format(class_names[int(classes[0][i])],
                                        np.array(scores[0][i]),
                                        np.array(boxes[0][i])))
    img = cv2.cvtColor(img_raw.numpy(), cv2.COLOR_RGB2BGR)
    img = draw_outputs(img, (boxes, scores, classes, nums), class_names)
    cv2.imwrite(output_path + 'detection.jpg', img)
    print('output saved to: {}'.format(output_path + 'detection.jpg'))
    
    # prepare image for response
    _, img_encoded = cv2.imencode('.png', img)
    response = img_encoded.tostring()
    
    #remove temporary image
    os.remove(image_name)

    try:
        return Response(response=response, status=200, mimetype='image/png')
    except FileNotFoundError:
        abort(404)
if __name__ == '__main__':
    app.run(debug=False, host = '0.0.0.0', port=5000)