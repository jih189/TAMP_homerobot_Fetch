import trimesh
import numpy as np

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

def generate_point_cloud(mesh, cameras):
    # Initialize GLUT
    glutInit()

    # Create a window for off-screen rendering
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB)
    glutInitWindowSize(1, 1)
    glutCreateWindow("Off-screen")

    # Set up OpenGL context
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE)

    # Create an empty list to store the points from each camera
    point_clouds = []

    for camera in cameras:
        # Set up the projection matrix
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(camera["fov"], camera["aspect_ratio"], camera["near_clip"], camera["far_clip"])

        # Set up the modelview matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(camera["position"][0], camera["position"][1], camera["position"][2],
          camera["target"][0], camera["target"][1], camera["target"][2],
          camera["up"][0], camera["up"][1], camera["up"][2])

        # Render the mesh
        glBegin(GL_POINTS)
        for vertex in mesh.vertices:
            glVertex3f(vertex[0], vertex[1], vertex[2])
        glEnd()

        # Read the depth buffer
        depth_buffer = glReadPixels(0, 0, camera["width"], camera["height"], GL_DEPTH_COMPONENT, GL_FLOAT)

        # Unpack the depth values
        depths = np.frombuffer(depth_buffer, dtype=np.float32)
        depths = depths.reshape((camera["height"], camera["width"]))

        # Compute the points for this camera and add them to the list
        points = camera_to_points(camera, depths)
        point_clouds.append(points)

    # Combine the point clouds from all cameras
    point_cloud = np.vstack(point_clouds)

    return point_cloud

def camera_to_points(camera, depths):
    # Compute the camera intrinsics matrix (K)
    f = (camera["width"] / 2) / np.tan(np.radians(camera["fov"] / 2))
    K = np.array([
        [f, 0, camera["width"] / 2],
        [0, f / camera["aspect_ratio"], camera["height"] / 2],
        [0, 0, 1]
    ])

    # Compute the camera extrinsics matrix (Rt)
    R, t = look_at(camera["position"], camera["target"], camera["up"])
    Rt = np.vstack((np.column_stack((R, t)), np.array([0,0,0,1.0])))

    # Initialize an empty list for the 3D points
    points = []

    # Iterate over the depth values
    for y in range(depths.shape[0]):
        for x in range(depths.shape[1]):
            # Convert depth value to linear depth
            linear_depth = camera["near_clip"] * camera["far_clip"] / (camera["far_clip"] - depths[y, x] * (camera["far_clip"] - camera["near_clip"]))

            # Convert the pixel coordinates (x, y) to normalized device coordinates (ndc_x, ndc_y)
            ndc_x = (x - camera["width"] / 2) / (camera["width"] / 2)
            ndc_y = -(y - camera["height"] / 2) / (camera["height"] / 2)

            # Compute the 3D point in the camera coordinate system
            point_cam = np.dot(np.linalg.inv(K), np.array([ndc_x * linear_depth, ndc_y * linear_depth, linear_depth]))

            # Compute the 3D point in the world coordinate system
            point_world = np.dot(np.linalg.inv(Rt), np.append(point_cam, 1))

            # Append the point to the list
            points.append(point_world[:3])

    return np.array(points)

def look_at(position, target, up):
    z_axis = np.array(position, dtype=float) - np.array(target, dtype=float)
    z_axis /= np.linalg.norm(z_axis)
    x_axis = np.cross(np.array(up, dtype=float), z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)

    R = np.array([x_axis, y_axis, z_axis]).T
    t = np.dot(-R , np.array(position, dtype=float))

    return R, t

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
