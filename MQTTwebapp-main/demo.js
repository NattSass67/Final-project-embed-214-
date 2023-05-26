function startConnect(){

    clientID = "clientID - "+parseInt(Math.random() * 100);

    //host = document.getElementById("host").value;   
    host= "mqtt-dashboard.com";
   // port = document.getElementById("port").value; 
    port="8000"; 
      

    document.getElementById("messages").innerHTML += "<span> Connecting...."+"</span><br>";
    //document.getElementById("messages").innerHTML += "<span> Using the client Id " + clientID +" </span><br>";

    client = new Paho.MQTT.Client(host,Number(port),clientID);

    client.onConnectionLost = onConnectionLost;
    client.onMessageArrived = onMessageArrived;

    client.connect({
        onSuccess: onConnect
//        userName: userId,
 //       passwordId: passwordId
    });


}


function onConnect(){
    //topic =  document.getElementById("topic_s").value;
    topic= "topic/out";

    document.getElementById("messages").innerHTML += "<span> Connected " + "</span><br>";
    client.subscribe(topic);
}



function onConnectionLost(responseObject){
    document.getElementById("messages").innerHTML += "<span> ERROR: Connection is lost.</span><br>";
    if(responseObject !=0){
        document.getElementById("messages").innerHTML += "<span> ERROR:"+ responseObject.errorMessage +"</span><br>";
    }
}

function onMessageArrived(message){
    console.log("OnMessageArrived: "+message.payloadString);
    document.getElementById("messages").innerHTML += "<span> "+""+" "+message.payloadString + "</span><br>";
}

function startDisconnect(){
    client.disconnect();
    document.getElementById("messages").innerHTML += "<span> Disconnected. </span><br>";




}


