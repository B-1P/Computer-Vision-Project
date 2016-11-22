import numpy as np
from keras.callbacks import TensorBoard
from keras.engine import Input
from keras.layers import Convolution2D, MaxPooling2D, UpSampling2D
from keras.models import Model
from keras.preprocessing import image as image_utils

MASKS_DIR = '/media/b1/My Book/CSC420Data/data_road/training/gt_image_road_mask/'
IMAGES_DIR = '/media/b1/My Book/CSC420Data/data_road/training/image_2/'


def load_images(dir, grayscale=False):
    imgs = []
    for i in image_utils.list_pictures(dir):
        imgs.append(image_utils.img_to_array(image_utils.load_img(i, grayscale=grayscale, target_size=(372, 1244))))
    return np.array(imgs)


if __name__ == '__main__':
    datagen = image_utils.ImageDataGenerator(
        featurewise_center=True,
        featurewise_std_normalization=True,
        rotation_range=20,
        width_shift_range=0.3,
        height_shift_range=0.3,
        horizontal_flip=True
    )
    x_data = load_images(IMAGES_DIR)
    datagen.fit(x_data)
    y_data = load_images(MASKS_DIR, grayscale=True)

    input_tensor = Input(shape=(372, 1244, 3))  # this assumes K.image_dim_ordering() == 'tf'

    x = Convolution2D(512, 3, 3, activation='relu', border_mode='same')(input_tensor)
    x = MaxPooling2D((2, 2), border_mode='same')(x)
    x = Convolution2D(256, 3, 3, activation='relu', border_mode='same')(x)
    x = MaxPooling2D((2, 2), border_mode='same')(x)
    x = Convolution2D(256, 3, 3, activation='relu', border_mode='same')(x)
    encoded = MaxPooling2D((2, 2), border_mode='same')(x)

    # at this point the representation is (8, 4, 4) i.e. 128-dimensional

    x = Convolution2D(256, 3, 3, activation='relu', border_mode='same')(encoded)
    x = UpSampling2D((2, 2))(x)
    x = Convolution2D(256, 3, 3, activation='relu', border_mode='same')(x)
    x = UpSampling2D((2, 2))(x)
    x = Convolution2D(512, 3, 3, activation='relu')(x)
    x = UpSampling2D((2, 2))(x)
    decoded = Convolution2D(1, 3, 3, activation='sigmoid', border_mode='same')(x)

    autoencoder = Model(input_tensor, decoded)
    autoencoder.compile(optimizer='adadelta', loss='binary_crossentropy', metrics=['accuracy'])

    autoencoder.fit_generator(datagen.flow(x_data, y_data, batch_size=8), samples_per_epoch=len(x_data), nb_epoch=100,
                              callbacks=[TensorBoard(log_dir='/tmp/autoencoder')])
    autoencoder.save('trained.h5p')
