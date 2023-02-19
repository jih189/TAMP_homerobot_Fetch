import subprocess

# kill running rosnodes with tf_echo in the name
# get the names of the nodes with tf_echo in the name
tf_nodes = subprocess.check_output("rosnode list | grep tf_echo", shell=True)
# split the names into a list
tf_nodes = tf_nodes.split()
# kill the nodes
for node in tf_nodes:
    # sanitize the node name
    node = node.decode("utf-8")
    node = node.replace("\n", "")

    # kill the node
    subprocess.call("rosnode kill " + node, shell=True)


