(function( window )
{

  var save = document.querySelector( ".save" ),
      //popupQR = document.querySelector( ".qr" ),
      popupMail = document.querySelector( ".mail" ),
      body = document.body,
      mask = document.createElement("div"), 
      activeNav,
      quit = document.querySelector("#quit");
  var clicked = false;
      
  mask.className = "mask";

  quit.addEventListener("click", function()
  {
    swal({
      title: "Är du säker?",
      text: "Din modell kommer nu att kastas!",
      type: "warning",
      showCancelButton: true,
      confirmButtonColor: "#DD6B55",
      confirmButtonText: "Ja, kasta den!",
      cancelButtonText: "Avbryt",
      closeOnConfirm: false
    },
    function(){
      swal("Kastad!", "Din modell är nu borta.. för alltid!", "success");
    });
  });
  
  save.addEventListener("click", function()
  {
  
    /*if(!clicked)
    {
      clicked = true;
      //classie.add(body, "qr-open");
      classie.add(body, "mail-open");
      document.body.appendChild(mask);
      //classie.remove(body, "qr-close");
      classie.remove(body, "mail-close");
      activeNav="save-open";  
    }
    
    else
    {
      clicked = false;
      //classie.add(body, "qr-close");
      classie.add(body, "mail-close");
      //classie.remove(body, "qr-open");
      classie.remove(body, "mail-open");
      
      document.body.appendChild(mask);
      activeNav="save-open";    
    }*/
  });
  /*
  popupQR.addEventListener("click", function()
  {
    classie.add(body, ".popupMail.is-visible ");
    document.body.appendChild(mask);
  });*/
  
  
  
 
})( window );

//alert("Hello World!");