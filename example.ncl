<?xml version="1.0" encoding="ISO-8859-1"?>
<ncl id="preparationExample" xmlns="http://www.ncl.org.br/NCL3.0/EDTVProfile">
  <head>
    <regionBase>
      <region id="reg1" width="100%" height="100%" zIndex="1"/>
      <region id="regSE" polar="30deg" azimuthal="45deg" width="5deg" height="5deg"/>
    </regionBase>

    <descriptorBase>
      <descriptor id="dHeat" region="regSE" explicitDur="17s"/>
      <descriptor id="dLoading" region="reg1"/>
      <descriptor id="dVideo" region="reg1"/>
    </descriptorBase>

    <connectorBase>
      <causalConnector id="onBeginStartPrep">
        <simpleCondition role="onBegin"/>
        <simpleAction role="startPreparation"/>
      </causalConnector>
      <causalConnector id="onEndPrepStopStart">
        <simpleCondition role="onEndPreparation"/>
        <simpleAction role="stop"/>
        <simpleAction role="start"/>
      </causalConnector>
      <causalConnector id="onBeginStart">
        <simpleCondition role="onBegin"/>
        <simpleAction role="start"/>
      </causalConnector>
      <causalConnector id="onBeginSet_var_duration_by">
        <connectorParam name="var"/>
        <connectorParam name="duration"/>
        <connectorParam name="by"/>
        <simpleCondition role="onBegin"/>
        <compoundAction operator="seq">
          <simpleAction role="set" value="$var" duration="$duration" by="$by"/>
        </compoundAction>
      </causalConnector>
    </connectorBase>
  </head>
  <body>
    <port id="pInit" component="loading"/>

    <media id="loading" src="samples/loading.ogv" descriptor="dLoading"/>

    <media id="heat" type="application/x-sensory-effect-HeatType" descriptor="dHeat"/>

    <media id="video" src="samples/sun.mp4" descriptor="dVideo">
    </media>

    <link xconnector="onBeginStartPrep">
      <bind role="onBegin" component="loading"/>
      <bind role="startPreparation" component="heat"/>
    </link>

    <link xconnector="onEndPrepStopStart">
      <bind role="onEndPreparation" component="heat"/>
      <bind role="stop" component="loading"/>
      <bind role="start" component="video"/>
    </link>

    <link xconnector="onBeginStart">
       <bind role="onBegin" component="video"/>
       <bind role="start" component="heat"/>
    </link>

    <link xconnector="onBeginSet_var_duration_by">
      <bind role="onBegin" component="heat"/>
      <bind role="set" component="heat" interface="azimuthal">
        <bindParam name="var" value="135"/>
        <bindParam name="duration" value="17s"/>
        <bindParam name="by" value="30"/>
      </bind>
    </link>

  </body>
</ncl>
