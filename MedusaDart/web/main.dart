// Copyright (c) 2015, <your name>. All rights reserved. Use of this source code
// is governed by a BSD-style license that can be found in the LICENSE file.

import 'dart:html';
import 'package:three/three.dart';
import 'dart:math' as Math;
import 'package:vector_math/vector_math.dart' hide Ray;

var camera, cameraTarget, scene, renderer;



void main() {
 init();
    renderer = new WebGLRenderer(antialias: true, alpha: false);
     renderer.setSize(window.innerWidth, window.innerHeight);


     renderer.gammaInput = true;
     renderer.gammaOutput = true;
     renderer.physicallyBasedShading = true;

     renderer.shadowMapEnabled = true;
     //renderer.shadowMapCullFace = CullFaceBack;
     renderer.setSize(window.innerWidth, window.innerHeight);

     container.append(renderer.domElement);
 render();
}

init() {

  var container = document.createElement('div');
  document.body.append(container);
 

  camera = new PerspectiveCamera(35.0, window.innerWidth / window.innerHeight, 1.0, 15.0);
  camera.position.setValues(3.0, 0.15, 3.0);

  cameraTarget = new Vector3(0.0, -0.25, 0.0);
  scene = new Scene();
  var loader = new OBJLoader();
  
  var plane =
        new Mesh(new PlaneGeometry(40.0, 40.0));
    plane.rotation.x = -Math.PI / 2;
    plane.position.y = -0.5;
    scene.add(plane);

  
  

    


    //window.onResize.listen(onWindowResize);
  
}



render() {
  
  
  camera.lookAt(cameraTarget);

  renderer.render(scene, camera);

}