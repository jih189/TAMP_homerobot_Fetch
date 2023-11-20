from sklearn import mixture
import pickle
import glob
import numpy as np
# import matplotlib.pyplot as plt

X = np.zeros((0, 7))
filenames = []

num_components = 0
paths = []
foldername = "/catkin_ws/GMM/empty_world_trajectory_data/env_000000/"
filenames = sorted(glob.glob("%s/*.p"%foldername))


# Create paths from trajectory data
for idx, filename in enumerate(filenames):
    if idx > 0 and idx%500 == 0:
        print(idx)
    with open(filename, "rb") as f:
        # path = pickle.load(f, encoding="latin1")['path']
        path = pickle.load(f)['path']
        paths.append(path)
        path_len = path.shape[0]
        if path_len > num_components:
            num_components = path_len
        X = np.append(X, path, axis=0)

# Fit a GMM using sklearn - number of components = max(path_length), initialization is kmeans by default
gmm = mixture.GaussianMixture(n_components=num_components, random_state=0).fit(X)

gmm_name = '../gmm/'
np.save(gmm_name + 'weights', gmm.weights_, allow_pickle=False)
np.save(gmm_name + 'means', gmm.means_, allow_pickle=False)
np.save(gmm_name + 'covariances', gmm.covariances_, allow_pickle=False)


edges = []
for idx, path in enumerate(paths):
    if idx % 500 == 0:
        print(idx)
    predicted_distributions = gmm.predict(path)
    path_len = len(path)
    for idx1, idx2 in zip(range(path_len), range(1, path_len)):
        dist1, dist2 = predicted_distributions[idx1], predicted_distributions[idx2]
        if dist1!=dist2:
            edges.append([dist1, dist2])

edges = np.array(edges)
unique, counts = np.unique(edges, return_counts=True, axis = 0)
probabilities = counts / np.sum(counts) # the probability of the edge e_i occuring amongst all edges
# plt.plot(counts)


# np.save("gmm/gmm_means.npy", gmm.means_)
# np.save("gmm/gmm_coviarance.npy", gmm.covariances_)
np.save("../gmm/edges.npy", unique)
np.save("../gmm/edge_probabilities.npy", probabilities)

# print(num_components)
