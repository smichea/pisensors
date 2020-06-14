
var width = 400;
var height = 400;
var container = document.getElementById("mycanvas");
var scene = new THREE.Scene();
var camera = new THREE.PerspectiveCamera( 75, width/height, 0.1, 1000 );

var renderer = new THREE.WebGLRenderer();
renderer.setSize( width, height );
container.appendChild( renderer.domElement );

var geometry = new THREE.BoxGeometry(01,1,1);
var material = new THREE.MeshBasicMaterial(  { color:0xff0ff0, vertexColors: THREE.FaceColors }  );
var probe = new THREE.Mesh( geometry, material );
probe.geometry.faces[ 0 ].color.setHex( 0xffff00 );
probe.geometry.faces[ 1 ].color.setHex( 0xffff00 );
probe.geometry.faces[ 2 ].color.setHex( 0xff00ff );
probe.geometry.faces[ 3 ].color.setHex( 0xff00ff );
probe.geometry.faces[ 4 ].color.setHex( 0x00ffff );
probe.geometry.faces[ 5 ].color.setHex( 0x00ffff );
probe.geometry.faces[ 6 ].color.setHex( 0x55ffff );
probe.geometry.faces[ 7 ].color.setHex( 0x55ffff );
probe.geometry.faces[ 8 ].color.setHex( 0xff55ff );
probe.geometry.faces[ 9 ].color.setHex( 0xff55ff );
probe.geometry.faces[ 10 ].color.setHex( 0x55ffff );
probe.geometry.faces[ 11 ].color.setHex( 0x55ffff );
scene.add( probe );
camera.position.z = 5;
renderer.render( scene, camera );
function renderProbe(x,y,z,gx,gy,gz){
   console.log("x="+x+",y="+y+",z="+z+",gx="+gx+",gy="+gy+",gz="+gz);
   probe.lookAt ( x, y, z);
   requestAnimationFrame( animate );
}
function animate(){
   renderer.render( scene, camera );
}
