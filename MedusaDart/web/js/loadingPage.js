//Medusa 3D-reconstruction
//This file handles the start up page
//classie.js is used to easily add and remove classes

(function( window )
{  
  
  window.onload = function onLoad() {
    var circle = new ProgressBar.Circle('#progress', {
      color: '#7EC0EA',
      trailColor: '#CBE6F7',
      strokeWidth: 7,
      duration: 2500,
      easing: 'easeInOut'
    });

    circle.set(0);

    /*setTimeout(function() {
      circle.animate(0.3);
    }, 1000);*/

    /*setTimeout(function() {
      circle.animate(0.4);
    }, 2000);

    setTimeout(function() {
      circle.animate(0.8);
    }, 7000);*/

    setTimeout(function() {
      circle.animate(1);
    }, 0); 

     setTimeout(function() {
      circle.animate(1);
      //fix, switch to mainPage to soon
      window.location.replace("mainPage.html"); 
    },4000);
  }; 

})( window );





