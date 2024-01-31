#!/usr/bin/env python3
# Sistemas Avançados de Visão Industrial (SAVI 22-23)
# Miguel Riem Oliveira, DEM, UA


import open3d as o3d
import numpy as np
import cv2

view = {
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : False,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 32.578899383544922, 11.709379196166992, 8.0947761535644531 ],
			"boundingbox_min" : [ -10.503261566162109, -13.69407844543457, -0.072889998555183411 ],
			"field_of_view" : 60.0,
			"front" : [ 0.68837046539939939, 0.48194925602995048, 0.54209871515987906 ],
			"lookat" : [ 21.904576878370001, 11.315327244771934, 2.6470701693075807 ],
			"up" : [ -0.52538907337510965, -0.18399339774963908, 0.8307302517439622 ],
			"zoom" : 0.12119999999999993
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}

def main():

    # --------------------------------------
    # Initialization
    # --------------------------------------

    transform_matrix = np.array([[-0.286486, -0.646401, -0.707171, 22.0158],
                            [-0.958053, 0.18732, 0.2169, 9.02334],
                            [-0.00462435, 0.751655, -0.65954, 1.45299],
                            [0,0,0,1]])
    
    opengl_to_opencv = np.array([[1,0,0,0],
                            [0,-1,0,0],
                            [0,0,-1,0],
                            [0,0,0,1]])
    
    # opengl_to_opencv = np.array([[1,0,0,0],
    #                             [0,1,0,0],
    #                             [0,0,1,0],
    #                             [0,0,0,1]]) # DEBUG

    composed_trans = np.dot(transform_matrix, opengl_to_opencv)
    
    translation = transform_matrix[:3, 3]

    filename = '/home/diogo/Desktop/mp3d_dataset/2azQ1b91cZZ/v1/scans/2azQ1b91cZZ/2azQ1b91cZZ/matterport_mesh/7812e14df5e746388ff6cfe8b043950a/7812e14df5e746388ff6cfe8b043950a.obj'
    print('Loading ' + filename)
    viewing_mesh = o3d.io.read_triangle_mesh(filename, True)
    model = o3d.io.read_triangle_model(filename, True)
    print(model)
    


    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
    sphere.translate(translation)
    sphere.paint_uniform_color([1,0,0])

    frame_world = o3d.geometry.TriangleMesh().create_coordinate_frame(size=0.5, origin=np.array([0., 0., 0.]))
    frame_world.transform(composed_trans)

    intrinsics_list = [1069.12, 0, 629.037, 0, 1069.6, 520.428, 0, 0, 1]

    K = np.zeros((3, 3))
    K[0, :] = intrinsics_list[0:3]
    K[1, :] = intrinsics_list[3:6]
    K[2, :] = intrinsics_list[6:9]

    width = 1280
    height = 1024

    # --------------------------------------
    # Execution
    # --------------------------------------
    
    entities = [viewing_mesh, sphere, frame_world]
    
    renderer = o3d.visualization.rendering.OffscreenRenderer(width, height)

    for mesh in model.meshes:
        mtl = model.materials[mesh.material_idx]
        mtl.base_color = [1.0, 1.0, 1.0, 1.0]  # RGBA
        mtl.shader = "defaultUnlit"
        renderer.scene.add_geometry(mesh.mesh_name, mesh.mesh, mtl)

    o3d_intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    renderer.setup_camera(o3d_intrinsics, np.linalg.inv(composed_trans))
    image = np.array(renderer.render_to_image())

    cv2.imwrite('image.png', image)
    # cv2.waitKey(0)
    
    o3d.visualization.draw_geometries(entities,
                                  zoom=view['trajectory'][0]['zoom'],
                                  front=view['trajectory'][0]['front'],
                                  lookat=view['trajectory'][0]['lookat'],
                                  up=view['trajectory'][0]['up'])
    
    # --------------------------------------
    # Termination
    # --------------------------------------


if __name__ == "__main__":
    main()
