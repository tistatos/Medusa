//Medusa 3D-reconstruction
//This file handles the start up page
//classie.js is used to easily add and remove classes

(function( window )
{
  
  var infoMenu = document.querySelector( ".infoMenu" ),
      infoButton = document.querySelector( "#infoButton" ),
      startButton = document.querySelector( "#startButton" ),
      continueButton = document.querySelector( "#continueButton" ),
      instructionsMenu = document.querySelector( "instructions"),
      body = document.body,
      mask = document.createElement("div"), 
      activeNav,
      activeNav2;
      
  mask.className = "mask";
  
  //Show information menu when the information button is pushed
  infoButton.addEventListener("click", function()
  {
    classie.add(body, "menu-open");
    document.body.appendChild(mask);
    activeNav2="menu-open";
  
  });
  
  //Close the information menu when the close button is pushed
  [].slice.call(document.querySelectorAll("#closeButton")).forEach(function(el,i){
    el.addEventListener( "click", function(){
      classie.remove( body, activeNav2 );
      activeNav2 = "";
      document.body.removeChild(mask);
    } );
  });

  //Slide down the start button when it's pushed
  //Slide in instruction about the scanning process after the start button has been pushed
  startButton.addEventListener("click", function()
  {
    classie.add(body, "startButton-open");
    classie.add(body, "instructions-open");
    document.body.appendChild(mask);
    activeNav="startButton-open";
  });

  //Slide down the instructions menu when continueButton is pushed
  //Slide in Progressbar when continueButton is pushed
  continueButton.addEventListener("click", function()
  {
    //window.location = "../mainPage.html";
    classie.add(body, "instructions-open2");
    classie.add(body, "inProgress-open");
    document.body.appendChild(mask);
    activeNav="instructions"; 
  });

})( window );