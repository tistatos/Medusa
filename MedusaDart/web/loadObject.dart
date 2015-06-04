library obj;
import 'dart:html';
import 'dart:math' as Math;
import 'package:three/three.dart';
import 'package:vector_math/vector_math.dart';
var container, stats;
Camera camera;
var scene, renderer, object;


var touchX = 0;
var mouseX = 0,
    mouseY = 0;
var windowHalfX = window.innerWidth / 2;
var windowHalfY = window.innerHeight / 2;

//Intitialize scene
init(String hash) {
  container = document.getElementById('left');
  document.body.append(container);

  //Camera
  camera = new PerspectiveCamera(35.0, window.innerWidth / window.innerHeight, 1.0, 2000.0);
  camera.position.z = 0.0;

  //Scene
  scene = new Scene();

  //Lights
  var ambient = new AmbientLight(0x101030);
  scene.add(ambient);

  var directionalLight = new DirectionalLight(0xffeedd);
  directionalLight.intensity = 1.0;
  directionalLight.position.setValues(-5.0, 0.0, 20.0);
  scene.add(directionalLight);


  /*
  //Texture
  var texture = new Texture();  
  var loaderTexture = new ImageLoader();
  
  loaderTexture.addEventListener('load', (event) 
  {
    texture.image = event.content;
    texture.needsUpdate = true;
  });
  
  loaderTexture.load('textures/UV_Grid_Sm.jpg');
*/

  //Ta bort sen

  var loader = new OBJLoader();
  loader.load('scans/' + hash + '.obj').then((object) {
    //object.position.y = -90.0;
    //object.position.z = -340.0;
    object.scale.y = 7.0;
    object.scale.x = 7.0;
    object.scale.z = 7.0;
    object.position.z = 0.0;
    object.rotation.y = Math.PI;
    object.rotation.z = Math.PI;
    object.name = "mesh";
    //object.applyMatrix(object.matrix.invert());
    scene.add(object);
    querySelector("#box").text = "";
  });
  //----------

  //Model
  /*
  var loader = new OBJLoader(useMtl:false);
  loader.load('obj/file.obj').then((object) 
  {
    object.children.forEach((Object3D e) 
     {
      e.children.forEach((Object3D obj) 
      {
        if (obj is Mesh) 
          (obj.material as MeshLambertMaterial).map = texture;
      });
    });
    object.position.y = -150.0;
    object.position.z = -150.0;
    scene.add(object);
  });
  
   */

  //Init renderer
  renderer = new WebGLRenderer(antialias: true, alpha: true);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setClearColorHex(0xFFFFFF, 1);
  container.append(renderer.domElement);
  document.onMouseMove.listen(onDocumentMouseMove);
  document.onTouchMove.listen(onDocumentTouchMove);
  document.onTouchStart.listen(onDocumentTouchStart);
}

//listen to mouse moveing
onDocumentMouseMove(MouseEvent event) {
  mouseX = (event.client.x - windowHalfX) / 2;
  mouseY = (event.client.y - windowHalfY) / 2;
}
onDocumentTouchStart(TouchEvent event) {
  touchX = event.touches.first.page.x;
}
//listen to mouse moveing
onDocumentTouchMove(TouchEvent event) {
  var touch = event.touches.first.page.x - touchX;
  mouseX += touch;
  touchX = event.touches.first.page.x;
}


animate(num time) {
  window.requestAnimationFrame(animate);
  render();
}

//Render function
render() {
  var r = 7;
  var theta = mouseX / 80;
  var x = r * Math.sin(theta);
  var z = r * Math.cos(theta);
  camera.position.x = x;
  camera.position.z = z;
  //camera.position.x += (mouseX - camera.position.x) * 0.05;
  //camera.position.y += (-mouseY - camera.position.y) * 0.05;
  camera.lookAt(scene.position);
  camera.up = new Vector3(0.0, 1.0, 0.0);

  var xconfig = 1;
  if (camera.rotation.y.abs() > Math.PI / 2) xconfig = -1;
  //camera.rotation.x= xconfig * Math.min(Math.max(-Math.PI/16,Math.atan(mouseY/100)),Math.PI/16);
  //var obj = scene.getObjectByID(1);
  //obj.rotation.y = Math.PI + mouseX;

  renderer.render(scene, camera);
}

void main() {
  Storage localStorage = window.localStorage;
  var hash = localStorage['hash'];
  print(hash);
  init(hash);

  animate(0);
}
