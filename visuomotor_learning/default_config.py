from easydict import EasyDict as edict

cfg = edict()

# original image size
cfg.img_w = 320
cfg.img_h = 240

cfg.vgg_mean = [103.939, 116.779, 123.68]  # in bgr

# joints data information
cfg.jnt_head_shape = [6]
cfg.jnt_arm_shape = [16]

# TODO check this
cfg.jnt_head_min = [30.0, 60.0, 55.0, 15.0, 52.0, 90.0]
cfg.jnt_head_max = [-40.0, -70.0, -55.0, -35.0, -50.0, 0.0]

cfg.jnt_head_min_real = [49.989, 22.005, 19.996, 15.0, 52.0, 90.0]
cfg.jnt_head_max_real = [-49.967, -36.984, -20.004, -35.0, -50.0, 0.0]

cfg.jnt_arm_min = [-95.0, 0.0, -37.0, 15.5, -90.0, -90.0, -20.0,
                   0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cfg.jnt_arm_max = [10.0, 160.8, 80.0, 106.0, 90.0, 0.0, 40.0,
                   60.0, 90.0, 90.0, 180.0, 90.0, 180.0, 90.0, 180.0, 270.0]

# real robot valueg
cfg.jnt_arm_min_real = [-95.5165, -0.021978, -30.0026, 15.044, -59.9994, -79.956, -19.978,
                        0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cfg.jnt_arm_max_real = [9.978025, 161.033, 74.9644, 106.033, 60.0, 25.011, 25.033,
                        60.0, 90.0, 90.0, 180.0, 90.0, 180.0, 90.0, 180.0, 270.0]


# dataset file/path
cfg.data_path = ''
cfg.dataset_dir = 'train'
cfg.test_dataset_dir = 'test'
cfg.background = 'data_kSL_background_right'
cfg.background_dir = cfg.background+'/images/calib'

cfg.imgs_left_file = 'images/left'
cfg.imgs_right_file = 'images/right'

cfg.imgs_calib_left_file = 'images/calib/left'
cfg.imgs_calib_right_file = 'images/calib/right'

cfg.l_arm_log = 'leftArm/data.log'
cfg.r_arm_log = 'rightArm/data.log'
cfg.head_log = 'head/data.log'

# training params visuomotor-learning
cfg.num_epochs = 200
cfg.batch_size = 160
cfg.buffer_size = 1500 # number of samples will be taken for shuffling, (> num_samples => shuffle all dataset)
# https://stackoverflow.com/questions/46444018/meaning-of-buffer-size-in-dataset-map-dataset-prefetch-and-dataset-shuffle

cfg.num_samples = 40000
cfg.test_num_samples = 8000
cfg.test_batch_size = 1

cfg.adam_lr = 0.0001
