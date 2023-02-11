#!/usr/bin/env python3

#print the python version
import sys
print(sys.version)

import tensorflow.compat.v1 as tf
import numpy as np
from CoM_prediction.CoMPmodel import *

# Global variables
BATCH_SIZE = 1
NUM_POINT = 7000
print("BASE_DIR: ", BASE_DIR)


class ModelWrapper():
    def __init__(self):
        # get the model
        self.pointclouds_pl = tf.placeholder(tf.float32, shape=(BATCH_SIZE, NUM_POINT, 4))
        self.labels_pl = tf.placeholder(tf.float32, shape=(BATCH_SIZE, NUM_POINT, 4))
        self.is_training_pl = tf.placeholder(tf.bool, shape=())

        # create the model
        self.pred, _ = get_model2(self.pointclouds_pl, self.is_training_pl, bn_decay=None)
        
        # resore session
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.allow_soft_placement = True
        config.log_device_placement = False
        self.sess = tf.Session(config=config)

        # restore the model
        saver = tf.train.import_meta_graph(os.path.join(BASE_DIR, 'log2', 'model.ckpt.index'))
        print("saver: ", saver)
        saver.restore(self.sess, os.path.join(BASE_DIR, 'log2', 'model.ckpt'))


    def predict(self, x):
        # get the prediction
        feed_dict = {self.pointclouds_pl: x, self.is_training_pl: True}
        pred_vals = self.sess.run([self.pred], feed_dict=feed_dict)[0]
        pred_val = pred_vals[0]
        # CoM translation
        pred_CoMs = (x[0,:,:3]- pred_val[:,:3]) * (1 - x[0,:,3:4])
        obj_point_count = np.sum(1-x[0,:,3:4])
        # average the predictions
        pred_CoM = np.sum(pred_CoMs, axis=0) / obj_point_count
        return pred_CoM