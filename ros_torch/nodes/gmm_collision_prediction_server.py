#!/usr/bin/env python3.8

# display the python version
import sys
print(sys.version)
import os
import rospy
from ros_tensorflow_msgs.srv import GMMCollisionPredict, GMMCollisionPredictResponse
from gmm_collision_prediction.model import ModelWrapper
from ros_point_cloud_utils import pointcloud2_to_xyz_array
from sklearn import mixture
import rospkg
import numpy as np

class RosInterface():
    def __init__(self):
        self.wrapped_model = ModelWrapper()
        self.predict_srv = rospy.Service('GMMCollisionPredict', GMMCollisionPredict, self.predict_cb)

        rospack = rospkg.RosPack()
        # Get the path of the desired package
        task_planner_package_path = rospack.get_path('task_planner')
        gmm1 = self.load_distribution(os.path.join(task_planner_package_path, "computed_gmms_dir/dl/dpgmm/"))
        gmm2 = self.load_distribution(os.path.join(task_planner_package_path, "computed_gmms_dir/dpgmm/"))
        self.gmm1_to_gmm2_map = self.map_large_gmm_to_small_gmm(gmm1, gmm2)

    
    def load_distribution(self, dir_name):

        means = np.load(dir_name + 'means.npy')
        covariances = np.load(dir_name + 'covariances.npy')
        # Create an sklearn Gaussian Mixture Model 
        sklearn_gmm = mixture.GaussianMixture(n_components = len(means), covariance_type='full')
        sklearn_gmm.precisions_cholesky_ = np.linalg.cholesky(np.linalg.inv(covariances))
        sklearn_gmm.weights_ = np.load(dir_name + 'weights.npy') # how common this distribution is.
        sklearn_gmm.means_ = means
        sklearn_gmm.covariances_ = covariances
        return sklearn_gmm
    
    def map_large_gmm_to_small_gmm(self, gmm1, gmm2):
        """
        gmm1 is a large gmm of ~1000 dist.
        gmm2 is a smaller gmm of ~100 distributions
        This maps which distribution in gmm2, that each distribution of gmm1 belongs to
        """
        return gmm1.predict(gmm2.means_)


    def predict_cb(self, req):
        rospy.loginfo("Prediction from service")
        pc_base_frame = pointcloud2_to_xyz_array(req.point_cloud)
        weights = self.wrapped_model.predict(pc_base_frame)
        weights = self.map_gmm2_weights_to_gmm1(weights)
        resp = GMMCollisionPredictResponse()
        resp.weights = []
        for weight in weights:
            resp.weights.append(weight)
        return resp
    
    def map_gmm2_weights_to_gmm1(self, weights):
        return weights[self.gmm1_to_gmm2_map]



def main():
    rospy.init_node("GMMCollisionPredict")
    rospy.loginfo("Creating the Torch Model")
    ri = RosInterface()
    rospy.loginfo("GMM Collision Prediction node initialized at service: GMMCollisionPredict")
    rospy.spin()

if __name__ == "__main__":
    main()
