(function( window )
{

  var save = document.querySelector( ".save" ),
      popupQR = document.querySelector( ".qr" ),
      popupMail = document.querySelector( ".mail" ),
      redo = document.querySelector(".redo"),
      quit = document.querySelector(".quit"),
      body = document.body,
      mask = document.createElement("div"), 
      activeNav;
  
  var clicked = false;
      
  mask.className = "mask";
  
  quit.addEventListener("click", function()
  {
    if(!clicked)
    {
      swal({
        title: "Är du säker på att du vill avsluta?",
        text: "Den nuvarande modellen kommer att kastas!",
        showCancelButton: true,
        confirmButtonColor: "#DD6B55",
        confirmButtonText: "Avsluta",
        closeOnConfirm: false
      },
      function(){
        //swal("Deleted!", "Your imaginary file has been deleted.", "success");
      });
     }
   });

  redo.addEventListener("click", function()
  {
    if(!clicked)
    {
      swal({
        title: "Är du säker på att du vill göra om modellen?",
        text: "Den nuvarande modellen kommer att kastas!",
        showCancelButton: true,
        confirmButtonColor: "#DD6B55",
        confirmButtonText: "Gör om!",
        closeOnConfirm: false
      },
      function(){
        window.location.replace("countDown.html"); 
      });
    }
  });

    /*
    if(!clicked)
    {
      clicked = true;
      classie.add(body, "qr-open");
      classie.add(body, "mail-open");
      document.body.appendChild(mask);
      classie.remove(body, "qr-close");
      classie.remove(body, "mail-close");
      activeNav="save-open";  
    }
    
    else
    {
      clicked = false;
      classie.add(body, "qr-close");
      classie.add(body, "mail-close");
      classie.remove(body, "qr-open");
      classie.remove(body, "mail-open");
      
      document.body.appendChild(mask);
      activeNav="save-open";    
    }*/

  /*
  popupQR.addEventListener("click", function()
  {
    classie.add(body, ".popupMail.is-visible ");
    document.body.appendChild(mask);
  });*/
 
})( window );

//alert("Hello World!");