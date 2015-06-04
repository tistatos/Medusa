//Medusa 3D-reconstruction
//This file handles the start up page
//classie.js is used to easily add and remove classes
var circle;
function initProgress()
{  
  //set up for the loadingbar
  circle = new ProgressBar.Circle('#progress', {
      color: '#7EC0EA',
      trailColor: '#CBE6F7',
      strokeWidth: 7,
      duration: 2500,
      easing: 'easeInOut'
    });

    //start at 0
    circle.set(0);
}

function increaseProgress()
{
  circle.animate(1);
}

function finish()
{
  circle.set(1);
}
