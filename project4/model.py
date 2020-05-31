# gets the log file dir as an input
# use model_chk.h5 result that has the smallest known validation loss

# for vis_utils please install pydot first with:
# pip install pydot
# python drive.py /home/workspace/model.h5 
# data_folder = '/opt/carnd_p3/data'
# pip install 'keras==2.0.9'
# sudo apt install ffmpeg


import argparse
import os
import csv
import keras
import cv2
import pickle
import numpy as np
import sklearn
from math import ceil
from pathlib import Path
from sklearn.model_selection import train_test_split
from keras.layers import Dense, Conv2D, Flatten, Dropout, MaxPooling2D, Input, Lambda
from keras.layers import GlobalAveragePooling2D, BatchNormalization, Activation, Cropping2D
from keras.models import load_model, model_from_json    
import matplotlib.pyplot as plt
import tensorflow as tf

IMG_HEIGHT, IMG_WIDTH = 160, 320
TOP_CROP = 64
batch_size=44
epochs=75

def plot_history(history, title, ix_number=-1, y_true=None, bins=50, accuracy=[], file_name=None):    
    
# plot train and validation loss
# history - dictionary
# loss, val_loss - keys in history for the plot
# accuracy=['accuracy', 'val_accuracy'] - keys for the accuracy plot
# ix_number - list index of the list values in history['pred'][ix_number] or the last one default
# pred - key and values list must match y_true length for the histogram
# bins - bins quantity for the hystogram
# file_name - file to save plot

  fig = plt.figure(figsize=(14,5))
  ax = fig.add_subplot(1, 2, 1)
  ax.set_title(title)
  ax.plot(history['loss'], label='Model loss')
  ax.plot(history['val_loss'], label='Model val_loss')
  ax.set_ylabel('loss')
  ax.set_xlabel('epoch')
  if len(accuracy)==0:
    ax.legend(loc='upper right') #['train','validation'], loc='upper right')
  elif len(accuracy)==2:
    ax2 = ax.twinx()
    ax2.plot(history[accuracy[0]], label='Train')
    ax2.plot(history[accuracy[1]], label='Validation')
    ax2.set_ylabel('accuracy')
    ax2.legend(loc='right') 

  ax = fig.add_subplot(1, 2, 2)
  ax.set_title(title)
  if 'pred' in history.keys():
    if ix_number==-1:
        ix = len(history['pred'])-1
    else:
        ix = ix_number
    if y_true is None:
        ax.hist(history['pred'][ix], label='prediction', bins=bins)
    else:
        #ax.hist([history['pred'][ix], y_true], label=['prediction', 'y_true'], bins=bins)
        ax.hist(np.concatenate((history['pred'][ix], y_true), axis=1), 
                label=['prediction', 'y_true'], 
                color=['red', 'lime'], 
                bins=bins)
        ax.legend(prop={'size': 10})
        
    ax.set_xlabel('target')
    print('predictions ', len(history['pred']))
  fig.tight_layout()
  if file_name is None:
    plt.show()
  else:
    plt.savefig(file_name)


def flip_images(X, y):
# horizontal flip an image, inverse steering angle
    if np.random.uniform()>0.5:
        X[:,:,:] = cv2.flip(X[:,:,:],1)
        y = -y
    return X, y

def generator(samples, data_path, batch_size=32, preproc=True, angle_corr=[0, 0.1, -0.1]):
    num_samples = len(samples)
    #angle_corr = [0, 0.1, -0.1] # center  left  right
    while 1: # Loop forever so the generator never terminates
        np.random.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                img_idx = np.random.randint(3) # random choice of center, left, right image index
                fname = os.path.join(data_path, batch_sample[img_idx].split('/')[-1])
                current_image = cv2.imread(fname)
                if current_image is None:
                    print('image not found{}'.format(fname))
                    continue
                if len(current_image) == 0:
                    print('image not found{}'.format(fname))
                    continue
                current_image = cv2.cvtColor(current_image, cv2.COLOR_BGR2RGB) # convert to RGB
                if preproc: # crop and resize an image, the custom layer will do the same instead
                    current_image = cv2.resize(current_image[TOP_CROP:, :], (IMG_HEIGHT, IMG_WIDTH), interpolation=cv2.INTER_CUBIC)

                center_angle = float(batch_sample[3]) + angle_corr[img_idx] # correct the angle for the left or right image
                current_image, center_angle = flip_images(current_image, center_angle)
                images.append(current_image)
                angles.append(center_angle)

            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)

def create_resnet_model(drop_rate=0.1, rnet_out_name=None, rnet_out_shape=None, dense_layers=[4096,512], top_crop=0, pooling='avg'):
    
    if top_crop > 0:
        inputs = Input(shape=(IMG_HEIGHT, IMG_WIDTH ,3), name='input_0')
        inputs = Cropping2D(cropping=((top_crop,0), (0,0)), name='croping_0')(inputs)
        inputs = Lambda(
            #lambda x: tf.image.resize(x / 255.0, (224, 224), method = tf.image.ResizeMethod.BICUBIC), # nearest bilinear bicubic cubic
            lambda x: tf.image.resize_images(x / 255.0, (224, 224), method = tf.image.ResizeMethod.BICUBIC), # nearest bilinear bicubic cubic
            name='resize_0')(inputs)
        base_model = keras.applications.ResNet50(weights= 'imagenet', include_top=False, pooling=pooling, # None or avg or max
                                                input_shape= (224, 224,3),
                                                input_tensor=inputs)
    else:
        base_model = keras.applications.ResNet50(weights= 'imagenet', include_top=False, pooling=pooling, # None or avg or max
                                                input_shape= (IMG_HEIGHT, IMG_WIDTH,3))

    if rnet_out_name is None:
        x = base_model.output # get the pretrained NN output layer when nothing specified
    else:
        try:
            x = base_model.get_layer(rnet_out_name).output # get layer by name
        except ValueError:
            x = None
            # get the last layer of the block, usually is it the Activation layer
            # each ResNet block reduces the input width and height by two, 
            # which allows you to find the last layer of the previous block
            # see the output of the plot_model()
            for layer in reversed(base_model.layers): 
                if layer.output.shape[-3:] == rnet_out_shape:
                    x = layer.output
                    print('layer.name: ', layer.name)
                    break
    if pooling is None:
        y = Flatten()(x) # no pooling layer, shapes: (None, h, w, fltr)->(None, h*w*n_fltr)
    else:
        if rnet_out_name is None:
            y = x # default pooling layer, shapes: (None, h, w, fltr)->(None, n_fltr)
        else:
            if pooling=='avg': # particular pooling layer, shapes: (None, h, w, fltr)->(None, n_fltr)
                y = GlobalAveragePooling2D()(x)
            else:
                y = GlobalMaxPooling2D()(x)

    for i, neurons in enumerate(dense_layers):
        y = Dense(neurons, activation='relu', name='dense_a_'+str(neurons)+'_'+str(i))(y)
        y = Dropout(drop_rate, name='drop_a_'+str(neurons)+'_'+str(i))(y)

    outputs_angle = Dense(1, name='pred_angle')(y)
    model = keras.Model(inputs=base_model.input, outputs=outputs_angle)
    return model

def read_log_file_2_list(log_file):
    samples = []
    with open(log_file) as csvfile:
        reader = csv.reader(csvfile)
        headers = next(reader) # first row with names
        print('Headers: ', headers)
        for line in reader:
            samples.append(line)
    return samples


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Training')
    parser.add_argument(
        'log_folder',
        type=str,
        nargs='?',
        default='',
        help='Path to IMG image folder. This is where the log file saved.'
    )
    args = parser.parse_args()

    log_file = os.path.join(args.log_folder, 'driving_log.csv')
    data_folder = os.path.join(args.log_folder, 'IMG')

    samples = read_log_file_2_list(log_file)

    train_samples, validation_samples = train_test_split(samples, test_size=0.2)


    # compile and train the model using the generator function
    train_generator = generator(train_samples, data_folder, batch_size=batch_size, preproc=False)
    validation_generator = generator(validation_samples, data_folder, batch_size=batch_size, preproc=False)

    model_chk_file = 'model_chk.h5'
    # Keras 2.0.9 does not know save_best_only option in the EarlyStopping()!!!
    callback_e = keras.callbacks.EarlyStopping(monitor='val_loss', patience=10) #, save_best_only=True   # val_accuracy
    callback_checkp = keras.callbacks.ModelCheckpoint(model_chk_file, monitor='val_loss', verbose=1, save_best_only=True)
    
    # reuse 'model_chk.h5' if the file exists otherwise rename for the new model
    #os.path.exists()
    if os.path.isfile(model_chk_file):
        model = load_model(model_chk_file, custom_objects={'tf':tf})
        print('model loaded from file {}'.format(model_chk_file))
    else:
        model = create_resnet_model(drop_rate=0.2, dense_layers=[512, 128, 32], pooling='avg', top_crop=TOP_CROP
                                    , rnet_out_name='activation_89', rnet_out_shape=(14,14,1024))
        model.compile(loss='mse', optimizer='adam', metrics=['mae'])
        print('new model created')
    #model.summary()
    keras.utils.plot_model(model, show_shapes=True, to_file='model.png')

    history = model.fit_generator(train_generator, 
                steps_per_epoch=ceil(len(train_samples)/batch_size), 
                validation_data=validation_generator, 
                validation_steps=ceil(len(validation_samples)/batch_size), 
                epochs=epochs, verbose=1, callbacks=[callback_e, callback_checkp])

    model.save('model.h5')
    history_dict = history.history.copy()
    plot_history(history_dict, 'train ResNet50', file_name='train.png') #os.path.join(backup_folder, 'train.png'))
    print("OK")
