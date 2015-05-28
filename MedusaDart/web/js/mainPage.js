//Medusa 3D-reconstruction
//mainPage.js works with the mainPage.html, mainPage.css and sweetalert.js to make the buttons more alive.

(function( window )
{
  //connect to the tags in the .html-file
  var save = document.querySelector( "#save" ),
      redo = document.querySelector("#reDo"),
      body = document.body,
      mask = document.createElement("div"), 
      activeNav,
      quit = document.querySelector("#quit");
      confirm = document.querySelector(".confirm");
  var clicked = false;
      
  mask.className = "mask";

  //if the save-button was clicked do this function
  //show a message and ask the user to write his/her e-mailadress
  save.addEventListener("click", function()
  {
    //overload the default with our own message in sweetalert
    swal({  title: "Maila mig min 3D-modell!",   
            text: "Skriv in din mailadress:",   
            type: "input",
            inputType: "email",
            showCancelButton: true,   
            closeOnConfirm: false,
            confirmButtonColor: "#7EC0EA",
            confirmButtonText: "Skicka",
            cancelButtonText: "Avbryt",  
            inputPlaceholder: "Din mail" }, 
            
            //validate the email-adress. 
            //If it is wrong pop up a message to the user and ask hem/her to rewrite the adress
            //if it is right, a confirmation message will pop up
            //if the user have not put in adress a meassage will pop up and say so
            function(inputValue){   
                                    //the format accepts dots in the mail-adress in the right places
                                    //and the characters after the last dot should be 2-3 char. long
                                    var mailformat = /^\w+([\.-]?\w+)*@\w+([\.-]?\w+)*(\.\w{2,3})+$/;
                                    if (inputValue === false) return false;      
                                    if (inputValue === "") 
                                    {     
                                      swal.showInputError("Du har inte fyllt i någon mailadress!");     
                                      return false   
                                    }
                                    if(!inputValue.match(mailformat))
                                    {
                                      swal.showInputError("Du har inte fyllt i en giltig mailadress!");     
                                      return false 
                                    }
                                    else
                                    {
                                      swal("Modellen är skickad till", inputValue); 
                                      save.disabled = true;
                                      save.style.backgroundColor = "#DAE6EF";
                                    }    

                                    });
    });
  
  //if the quit-button have been clicked, pop up a message to the user to confirm
  //if no, do nothing and go back to mainPage and the 3D-model If yes, pop up a new message
  //and tell the user that the model has been through away
  quit.addEventListener("click", function()
  {
    swal(
    {
      title: "Är du säker?",
      text: "Din modell kommer nu att kastas om du inte har sparat den!",
      showCancelButton: true,
      confirmButtonColor: "#7EC0EA",
      confirmButtonText: "Ja, kasta den!",
      cancelButtonText: "Avbryt",
      closeOnConfirm: false
    },
    function()
    {
      swal(
      {
        title: "Din modell har nu kastats!",
        type: "success",
        confirmButtonColor: "#7EC0EA",
        confirmButtonText: "OK!"
      },
        function()
        {
          window.location.replace("index.html");
        });
    });
  });

  //A pop upp with the question to the user if the user is sure to do a redo.
  //The model is going to through away if the user says yes.
  redo.addEventListener("click", function()
  {
      swal({
        title: "Är du säker på att du vill göra om modellen?",
        text: "Den nuvarande modellen kommer att kastas!",
        showCancelButton: true,
        confirmButtonColor: "#7EC0EA",
        confirmButtonText: "Gör om!",
        cancelButtonText: "Avbryt",
        closeOnConfirm: false
      },
      function(){
        window.location.replace("countDown.html"); 
      });
  });

 
})( window );

//alert("Hello World!");