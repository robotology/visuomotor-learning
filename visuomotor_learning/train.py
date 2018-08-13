#!/usr/bin/env python

from __future__ import print_function
from __future__ import print_function
import os
import glob
import tensorflow as tf
import argparse
import numpy as np
from datetime import datetime

from keras.applications.vgg19 import VGG19
from keras.layers import Dense, GlobalAveragePooling2D, Input, concatenate, \
    BatchNormalization, Activation
from keras import initializers
from keras.models import Model
from keras import backend as K
from keras.callbacks import TensorBoard
from keras.callbacks import ModelCheckpoint
from keras.callbacks import LearningRateScheduler
from keras import optimizers
import keras

import default_config

cfg = default_config.cfg

img_w = cfg.img_w
img_h = cfg.img_h

img_w_resize = 160
img_h_resize = 120

show_image = True

adam = optimizers.Adam(lr=0.0001)  # decay value will be changed by args.adam_decay


def dataset_input_fn(data_path, _batch_size, _num_epochs, _buffer_size=cfg.buffer_size,
                     _img_h_resized = cfg.img_h, _img_w_resized = cfg.img_w,
                     mode='train', label_type='arm', _parallel=1, img_norm_type='subtract_vgg',
                     _img_h_new=None, _img_w_new=None, _robot='sim'):
    """
    Read tfrecords of features and labels. Features are composed of 2 images ('img_l' and 'img_r') and array of jnt_head
    value (1x6). Labels is jnt_arm value(1x7).
    Images are firstly loaded as tf.float32, then subtracted with VGG mean value.
    Note: all images loaded are in BGR
    :param _img_h_resized: resized imaged height
    :param _img_w_resized: resized imaged width
    :param _robot:
    :param _img_h_new: new image height (if set to not None value)
    :param _img_w_new: new image width (if set to not None value)
    :param _parallel: number of parallel calls, see the tf.data.Dataset
    :param _buffer_size:
    :param data_path: full path to data tfrecord file
    :param _batch_size:
    :param _num_epochs: None for repeat input indefinitely
    :param mode: train/val/test
    :param label_type: type of label which is used (arm/arm_hand)
    :param img_norm_type: type of image normalization method (subtract_vgg/-1to1)
    :return: _features (dict of left images, shaped (None, height, width, channels), right images and tensor of head joints), _labels (tensor of arm joints) for
     a batch
    """

    def _parser(record):
        _feature = {'img_l': tf.FixedLenFeature([], tf.string),
                    'img_r': tf.FixedLenFeature([], tf.string),
                    'jnt_head': tf.FixedLenFeature(shape=cfg.jnt_head_shape, dtype=tf.float32),
                    'label': tf.FixedLenFeature(shape=cfg.jnt_arm_shape, dtype=tf.float32)}

        parsed = tf.parse_single_example(record, features=_feature)

        # Convert the image data from string back to the numbers
        img_l = tf.decode_raw(parsed['img_l'], tf.float32)
        img_r = tf.decode_raw(parsed['img_r'], tf.float32)

        # Get head joint
        jnt_head = tf.convert_to_tensor(parsed['jnt_head'], dtype=tf.float32)

        # label
        if label_type == 'arm':
            label = tf.convert_to_tensor(parsed['label'][:7], dtype=tf.float32)
        elif label_type == 'arm_hand':
            label = tf.convert_to_tensor(parsed['label'], dtype=tf.float32)
        else:
            raise ValueError("wrong norm_type!:" + label_type)

        # Reshape image data into the original shape before tfrecords packaging
        img_l = tf.reshape(img_l, [_img_h_resized, _img_w_resized, 3])
        img_r = tf.reshape(img_r, [_img_h_resized, _img_w_resized, 3])

        # Normalize image
        img_l = get_normalized_image(img_l, img_norm_type)
        img_r = get_normalized_image(img_r, img_norm_type)

        if _robot=='sim':
            jnt_head_min = cfg.jnt_head_min
            jnt_head_max = cfg.jnt_head_max
            jnt_arm_min = cfg.jnt_arm_min
            jnt_arm_max = cfg.jnt_arm_max
        elif _robot=='real':
            jnt_head_min = cfg.jnt_head_min_real
            jnt_head_max = cfg.jnt_head_max_real
            jnt_arm_min = cfg.jnt_arm_min_real
            jnt_arm_max = cfg.jnt_arm_max_real
        else:
            raise ValueError("wrong robot!:" + _robot)

        # Normalize joint value
        if label_type == 'arm':
            jnt_head = get_normalized_part(jnt_head, jnt_head_min[:7], jnt_head_max[:7])
            label = get_normalized_part(label, jnt_arm_min[:7], jnt_arm_max[:7])
        elif label_type == 'arm_hand':
            jnt_head = get_normalized_part(jnt_head, jnt_head_min, jnt_head_max)
            label = get_normalized_part(label, jnt_arm_min, jnt_arm_max)

        if _img_h_new is not None and _img_w_new is not None:
            img_l = tf.image.resize_images(img_l,
                                           [_img_h_new, _img_w_new])
            img_r = tf.image.resize_images(img_r,
                                           [_img_h_new, _img_w_new])

        return {"img_l": img_l, "img_r": img_r, "jnt_head": jnt_head}, label

    # dataset = tf.contrib.data.TFRecordDataset(data_path)  # old dataset, DEPRECATED in future
    dataset = tf.data.TFRecordDataset(data_path)
    dataset = dataset.map(_parser, num_parallel_calls=_parallel)

    dataset = dataset.batch(_batch_size)
    dataset = dataset.repeat(_num_epochs)  # with no arguments will repeat the input indefinitely
    if mode == 'train':
        dataset = dataset.shuffle(buffer_size=_buffer_size)

    iterator = dataset.make_one_shot_iterator()

    _features, _labels = iterator.get_next()

    return _features, _labels


def get_normalized_image(raw_image, norm_type):
    if norm_type == 'subtract_vgg':
        vgg_mean_tf = tf.constant(cfg.vgg_mean, dtype=tf.float32)
        vgg_mean_tf = tf.reshape(vgg_mean_tf, [1, 1, 3])
        reshaped_image = raw_image - vgg_mean_tf
    elif norm_type == '-1to1':
        reshaped_image = raw_image / 127.5 - 1.0  # tf.ones_like(raw_image)
    elif norm_type == '0to1':
        reshaped_image = raw_image / 255.0
    else:
        raise ValueError("wrong norm_type!:" + norm_type)

    return reshaped_image


def get_normalized_image_np(raw_image, _resize_image=1, norm_type='subtract_vgg'):
    reshaped_image = raw_image.copy(). \
        reshape(int(cfg.img_h / _resize_image),
                int(cfg.img_w / _resize_image), 3, order='F')  # .astype(np.float)

    if norm_type == 'subtract_vgg':
        reshaped_image[:, :, 0] = reshaped_image[:, :, 0] - cfg.vgg_mean[0]
        reshaped_image[:, :, 1] = reshaped_image[:, :, 1] - cfg.vgg_mean[1]
        reshaped_image[:, :, 2] = reshaped_image[:, :, 2] - cfg.vgg_mean[2]
    elif norm_type == '-1to1':
        reshaped_image[:, :, 0] = reshaped_image[:, :, 0] / 127.5 - 1.0
        reshaped_image[:, :, 1] = reshaped_image[:, :, 1] / 127.5 - 1.0
        reshaped_image[:, :, 2] = reshaped_image[:, :, 2] / 127.5 - 1.0
    elif norm_type == '0to1':
        reshaped_image[:, :, 0] = reshaped_image[:, :, 0] / 255.0
        reshaped_image[:, :, 1] = reshaped_image[:, :, 1] / 255.0
        reshaped_image[:, :, 2] = reshaped_image[:, :, 2] / 255.0
    else:
        raise ValueError("wrong norm_type!:" + norm_type)

    return reshaped_image


def get_original_image_np(normalized_image, norm_type='subtract_vgg'):
    """
    Un-normalize the image which has been normalized by some method, currently support "subtract_vgg" only
    :param normalized_image: numpy image, get from TFrecord
    :param norm_type: type of normalization, supported "subtract_vgg"
    :return: original image in np.float32
    """
    original_image = normalized_image.copy()
    if norm_type == 'subtract_vgg':
        original_image[:, :, 0] = original_image[:, :, 0] + cfg.vgg_mean[0]
        original_image[:, :, 1] = original_image[:, :, 1] + cfg.vgg_mean[1]
        original_image[:, :, 2] = original_image[:, :, 2] + cfg.vgg_mean[2]
    elif norm_type == '-1to1':
        original_image = (normalized_image + 1.0)*127.5
    else:
        raise ValueError("wrong norm_type!:" + norm_type)

    return original_image


def get_normalized_part(jnt_part, min_part, max_part):
    max_part_tf = tf.constant(max_part, dtype=tf.float32)
    min_part_tf = tf.constant(min_part, dtype=tf.float32)

    # return tf.div(jnt_part-min_part_tf, max_part_tf-min_part_tf)
    # return (jnt_part - min_part_tf) / 180.0
    return (jnt_part - (max_part_tf + min_part_tf) / 2.0) / 180.0


def get_normalized_part_np(jnt_part, min_part, max_part):
    # return (jnt_part - np.array(min_part))/180.0
    min_part_np = np.array(min_part)
    max_part_np = np.array(max_part)
    return (jnt_part - (max_part_np + min_part_np) / 2.0) / 180.0


def get_origin_part_np(normalized_part, min_part, max_part):
    # return 180.0 * normalized_part + np.array(min_part)
    min_part_np = np.array(min_part)
    max_part_np = np.array(max_part)
    return 180.0 * normalized_part + (max_part_np + min_part_np) / 2.0


def loss_sum_square_error(y_true, y_pred):
    return K.sum(K.square(y_pred - y_true), axis=-1)


def model_kinematic_structure(_features=None, _labels=None, label_type='arm', mode='train',
                              img_h_resized=cfg.img_h, img_w_resized=cfg.img_w):
    kern_init = initializers.glorot_normal()

    # train and test mode by default using dataset pipeline of Tensorflow
    if mode == 'train' or mode == 'test':
        img_input_l = Input(tensor=_features['img_l'], name='img_input_L')
        img_input_r = Input(tensor=_features['img_r'], name='img_input_R')
        jnt_head_input = Input(tensor=_features['jnt_head'], name='jnt_head_input')
    # test_np mode serves for the case features and sample are fed manually
    elif mode == 'test_np':
        img_input_l = Input(name='img_input_L',
                            shape=(img_h_resized, img_w_resized, 3))
        img_input_r = Input(name='img_input_R',
                            shape=(img_h_resized, img_w_resized, 3))
        jnt_head_input = Input(name='jnt_head_input',
                               shape=(6,))
    else:
        raise Exception('mode should be train, test or test_np only!!!')

    # create the base pre-trained model
    base_model_l = VGG19(input_tensor=img_input_l, weights='imagenet', include_top=False)
    base_model_r = VGG19(input_tensor=img_input_r, weights='imagenet', include_top=False)
    for layer_L in base_model_l.layers[1:]:
        layer_L.name = 'layer_L_' + layer_L.name
    for layer_R in base_model_r.layers[1:]:
        layer_R.name = 'layer_R_' + layer_R.name

    # add a global spatial average pooling layer
    x_l = base_model_l.output
    x_l = GlobalAveragePooling2D()(x_l)
    x_r = base_model_r.output
    x_r = GlobalAveragePooling2D()(x_r)

    # let's add a fully-connected layer
    x_l = Dense(1024, kernel_initializer=kern_init, name='dense_l')(x_l)
    x_l = BatchNormalization()(x_l)
    x_l = Activation('relu', name='activation_l')(x_l)

    x_r = Dense(1024, kernel_initializer=kern_init, name='dense_r')(x_r)
    x_r = BatchNormalization()(x_r)
    x_r = Activation('relu', name='activation_r')(x_r)

    x = concatenate([x_l, x_r])
    x = Dense(512, kernel_initializer=kern_init, name='dense_images')(x)
    x = concatenate([x, jnt_head_input])

    x = Dense(512, kernel_initializer=kern_init, name='dense_images_head')(x)

    x = Dense(256,  # try different activations
              kernel_initializer=kern_init, name='dense_images_head_2')(x)

    if label_type == 'arm':
        jnt_arm_pred = Dense(7, kernel_initializer=kern_init, name='pred_arm', activation='tanh')(x)
    elif label_type == 'arm_hand':
        jnt_arm_pred = Dense(16, kernel_initializer=kern_init, name='pred_arm', activation='tanh')(x)
    else:
        raise Exception('Unknown label_type')

    # this is the model we will train
    model = Model(inputs=[img_input_l, img_input_r, jnt_head_input], outputs=jnt_arm_pred)
    # compile the model (should be done *after* setting layers to non-trainable)
    if mode == 'train':
        model.compile(target_tensors=[_labels],
                      optimizer=adam, metrics=['accuracy'],
                      loss='mean_squared_error')
    elif mode == 'test' or mode == 'test_np':
        print('test mode need compile model outside this')
    else:
        raise Exception('mode should be train, test or test_np only!!!')
    return model


class Histories(keras.callbacks.Callback):
    def __init__(self, _log_dir, _val_model, train_weight_path, _val_label):
        super(Histories, self).__init__()
        self.log_dir = _log_dir
        self.aucs = []
        self.losses = []
        self.val_acc = []
        self.val_model = _val_model
        self.trained_weight_path = train_weight_path
        self.val_labels = _val_label

    def on_epoch_end(self, epoch, logs={}):
        self.losses.append(logs.get('loss'))

        if not epoch % 30:
            self.model.save(self.log_dir + '/' + ''.join(args.train_dataset) + '_' + str(epoch) + '.h5')
        lr = K.get_value(self.model.optimizer.lr)

        self.model.save(self.trained_weight_path)

        self.val_model.load_weights(self.trained_weight_path)

        self.val_model.compile(optimizer=adam, metrics=['accuracy'],
                               target_tensors=[self.val_labels],
                               loss='mean_squared_error')

        scores = self.val_model.evaluate(x=None, y=None, batch_size=None,
                                         steps=int(args.test_num_samples/args.batch_size),
                                         verbose=1)

        self.val_acc.append(scores[1])
        print('Validation accuracy: {0}'.format(scores))
        print('End of {:d} epoch evaluation'.format(epoch))
        return


class LearningRateCallback(keras.callbacks.Callback):
    def on_epoch_begin(self, epoch, logs=None):
        lr = K.get_value(self.model.optimizer.lr)
        decay = K.get_value(self.model.optimizer.decay)
        print('decay: {0}'.format(decay))
        if not decay > 0. and epoch > 0:
            iter = K.get_value(self.model.optimizer.iterations)
            my_decay = np.sqrt(epoch)
            lr_decay = lr / my_decay
            print('iterations: {0}'.format(iter))
            K.set_value(self.model.optimizer.lr, lr_decay)
            print('decay learning rate: {0}'.format(lr_decay))


def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='Train and Test Kinematic Structure Learning model')
    parser.add_argument('--train', dest='train_dataset', help='Name of the train dataset folder, the dataset file will '
                                                              'have default name as train.tfrecords',
                        default=cfg.dataset_dir)
    parser.add_argument('--test', dest='test_dataset', help='Name of the test dataset folder, the dataset file will '
                                                            'have default name as train.tfrecords',
                        default=cfg.test_dataset_dir)
    parser.add_argument('--buffer', dest='buffer_size', help='Buffer size for Tensorflow dataset, the higher, '
                                                             'the better',
                        default=cfg.buffer_size, type=int)
    parser.add_argument('--batch', dest='batch_size', help='Batch size for training, the higher, the better',
                        default=cfg.batch_size, type=int)
    parser.add_argument('--samples', dest='num_samples', help='Number of sample of training dataset',
                        default=cfg.num_samples, type=int)
    parser.add_argument('--epoch', dest='num_epochs', help='Number of epochs of training ',
                        default=cfg.num_epochs, type=int)
    parser.add_argument('--parallel', dest='num_parallel', help='Number of parallel calls for tensorflow dataset ',
                        default=1, type=int)
    parser.add_argument('--samples_test', dest='test_num_samples', help='Number of sample of testing dataset',
                        default=cfg.test_num_samples, type=int)
    parser.add_argument('--img_h_resized', dest='img_h_resized', help='Change the size of image height to new one',
                        default=120, type=int)
    parser.add_argument('--img_w_resized', dest='img_w_resized', help='Change the size of image width to new one',
                        default=120, type=int)
    parser.add_argument('--decay', dest='adam_decay', help='Change the decay value of adam optimizer. If this value is '
                                                           'set to 0.0, the learning rate will directly reduced by '
                                                           'sqrt(t=epoch) every epoch',
                        default=0.0, type=float)
    parser.add_argument('--resume', dest='resume', help='Resume training', default=False, type=bool)
    parser.add_argument('--saved', dest='saved_model', help='Saved data file to resume training, only effected with '
                                                            'resume is True',
                        default=None)
    parser.add_argument('--lr', dest='adam_lr', help='Learning rate value of adam optimizer, only effected with resume'
                                                     'is True',
                        default=cfg.adam_lr, type=float)
    parser.add_argument('--robot', dest='robot', help='Type of robot data (sim/real)',
                        default='sim')

    _args = parser.parse_args()

    return _args


def get_dataset_filename(_dataset_name):
    processed_folder = os.path.join(os.getcwd(), 'processed_data', _dataset_name)
    _dataset_filename = glob.glob('{}/*.tfrecords'.format(processed_folder))
    return _dataset_filename


def train(_train_file, _test_file, _buffer_size,
          _batch_size, _num_samples, _num_epochs, img_h_resized, img_w_resized,
          _test_batch_size=cfg.test_batch_size, _test_num_samples=cfg.test_num_samples,
          _resize_image=1, _parallel=1, _robot='sim'):
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    config.gpu_options.visible_device_list = "0"

    sess = tf.Session(config=config)
    K.set_session(sess)

    init_op = tf.global_variables_initializer()

    sess.run(init_op)

    print('=========================== START TRAINING =============================')
    print('Preparing dataset. Please wait...')
    now = datetime.now().strftime('_%Y%m%d_%H%M')
    dataset_time = ''.join(args.train_dataset) + '_time' + now
    log_dir = './log/' + dataset_time

    print('log_dir: {:s}'.format(log_dir))

    features, labels = dataset_input_fn(_train_file, _batch_size, _buffer_size=_buffer_size,
                                        _num_epochs=None, _parallel=_parallel,
                                        _img_h_resized=img_h_resized, _img_w_resized=img_w_resized, _robot=_robot)
    tb_callback = TensorBoard(log_dir=log_dir,
                              histogram_freq=0, write_graph=True, write_images=True)
    weights_improved_path = log_dir + '/weights-improvement-{epoch:02d}-{acc:.2f}.hdf5'
    checkpoint = ModelCheckpoint(weights_improved_path, monitor='acc', verbose=1, save_best_only=True, mode='max')
    # histories = Histories(log_dir)
    K.set_value(adam.decay, args.adam_decay)
    if not args.resume:
        learning_model = model_kinematic_structure(features, labels)
    else:
        learning_model = model_kinematic_structure(features, labels, mode='test')
        print('load weights to resume from {0}'.format(args.saved_model))
        learning_model.load_weights(args.saved_model)
        K.set_value(adam.lr, args.adam_lr)
        learning_model.compile(target_tensors=[labels],
                               optimizer=adam, metrics=['accuracy'],
                               loss='mean_squared_error')

    val_features, val_labels = dataset_input_fn(_test_file, _test_batch_size,
                                                _num_epochs=None,
                                                _img_h_resized=img_h_resized, _img_w_resized=img_w_resized,
                                                mode='test', _robot=_robot)
    val_model = model_kinematic_structure(val_features, val_labels, mode='test')
    _trained_weight_path = log_dir + '/weight_' + dataset_time + '.h5'
    histories = Histories(log_dir, val_model, _trained_weight_path, val_labels)

    learning_model.summary()
    learning_model.fit(steps_per_epoch=int(_num_samples / _batch_size),
                       epochs=_num_epochs, verbose=1,
                       callbacks=[tb_callback, checkpoint, histories])  # lr_callback

    trained_model_path = log_dir + '/' + dataset_time + '.h5'
    learning_model.save(trained_model_path)
    # _trained_weight_path = log_dir + '/weight_' + dataset_time + '.h5'
    learning_model.save(_trained_weight_path)
    K.clear_session()

    # Second Session to test loading trained model without tensors
    print('=========================== FINISH TRAINING, START TESTING =============================')
    print('Preparing test dataset. Please wait...')

    test_features, test_labels = dataset_input_fn(_test_file, _test_batch_size,
                                                  _num_epochs=None,
                                                  _img_h_resized=img_h_resized, _img_w_resized=img_w_resized,
                                                  mode='test', _robot=_robot)
    test_model = model_kinematic_structure(test_features, test_labels, mode='test')

    test_model.load_weights(_trained_weight_path)

    test_model.compile(optimizer=adam, metrics=['accuracy'],
                       target_tensors=[test_labels],
                       loss='mean_squared_error')

    test_model.summary()

    scores = test_model.evaluate(x=None, y=None, batch_size=None,
                                 steps=int(_test_num_samples / _test_batch_size),
                                 verbose=1)

    print('test accuracy: {0}'.format(scores))


if __name__ == '__main__':
    args = parse_args()
    print(args)

    train_filename = get_dataset_filename(args.train_dataset)
    test_filename = get_dataset_filename(args.test_dataset)

    print('train filenames:', train_filename)
    print('test filenames:', test_filename)

    train(train_filename, test_filename, args.buffer_size,
          args.batch_size, args.num_samples, args.num_epochs, args.img_h_resized, args.img_w_resized,
          _test_num_samples=args.test_num_samples,
          _parallel=args.num_parallel,
          _robot=args.robot)

