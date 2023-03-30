import trimesh
import numpy as np

def create_axis_arrow(axis, shaft_radius=0.05, shaft_length=1.0, head_radius=0.1, head_length=0.2, color=None):
    # Create the shaft (cylinder) and head (cone) of the arrow
    shaft = trimesh.creation.cylinder(radius=shaft_radius, height=shaft_length)
    head = trimesh.creation.cone(radius=head_radius, height=head_length)

    shaft.apply_translation([0,0,shaft_length/2.0])

    # Move the head to the end of the shaft
    head.apply_translation([0, 0, shaft_length])

    # Combine the shaft and head to create the arrow
    arrow = trimesh.util.concatenate(shaft, head)

    # Align the arrow with the specified axis
    if axis == 'x':
        arrow.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    elif axis == 'y':
        arrow.apply_transform(trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0]))

    # Set the color of the arrow
    if color is not None:
        arrow.visual.vertex_colors = color

    return arrow

def create_axis_arrows(x_length=1.0, y_length=1.0, z_length=1.0):
    axislist = []
    x_arrow = create_axis_arrow('x', shaft_length=x_length, color=[255, 0, 0, 255])
    y_arrow = create_axis_arrow('y', shaft_length=y_length, color=[0, 255, 0, 255])
    z_arrow = create_axis_arrow('z', shaft_length=z_length, color=[0, 0, 255, 255])
    axislist.append(x_arrow)
    axislist.append(y_arrow)
    axislist.append(z_arrow)
    return axislist

def sample_points_on_mesh(mesh, num_points):
    # Calculate the area of each face
    face_areas = mesh.area_faces

    # Normalize the face areas to create a probability distribution
    face_probs = face_areas / face_areas.sum()

    # Sample face indices based on their probabilities
    sampled_face_indices = np.random.choice(len(mesh.faces), size=num_points, p=face_probs)

    # Sample barycentric coordinates for each point
    u = np.random.rand(num_points, 1)
    v = np.random.rand(num_points, 1)
    out_of_range = u + v > 1
    u[out_of_range] = 1 - u[out_of_range]
    v[out_of_range] = 1 - v[out_of_range]
    w = 1 - u - v

    # Calculate the 3D Cartesian coordinates of the sampled points
    vertices = mesh.vertices[mesh.faces[sampled_face_indices]]
    sampled_points = u * vertices[:, 0] + v * vertices[:, 1] + w * vertices[:, 2]

    return sampled_points
