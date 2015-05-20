library obj;
import 'dart:html';
import 'dart:math' as Math; 
import 'package:vector_math/vector_math.dart';
import 'package:three/three.dart';




var container, stats;

var camera, scene, renderer, object;

var mouseX = 0,
    mouseY = 0;
var windowHalfX = window.innerWidth / 2;
var windowHalfY = window.innerHeight / 2;

//Intitialize scene
init()
{
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
    loader.load('obj/file.obj').then((object) {
      //object.position.y = -90.0;
      //object.position.z = -340.0;
      object.position.z = -5.0;
      object.rotation.y = -Math.PI/2;
      
      scene.add(object);
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
  renderer.setClearColorHex( 0xF5F5F5, 1 );
  container.append(renderer.domElement);
  document.onMouseMove.listen(onDocumentMouseMove);  
}

//listen to mouse moveing
onDocumentMouseMove(MouseEvent event) 
{
  mouseX = (event.client.x - windowHalfX) / 2;
  mouseY = (event.client.y - windowHalfY) / 2;
}


animate(num time)
{
  window.requestAnimationFrame(animate);
  render();
}

//Render function
render()
{
  //camera.position.x += (mouseX - camera.position.x) * 0.05;
  //camera.position.y += (-mouseY - camera.position.y) * 0.05;
  camera.lookAt(scene.position);
  
  //object.rotation.y += 0.5;
  //object is null in render loop????
  
  renderer.render(scene, camera);
}

void main() 
{  
  init();
  animate(0);
}