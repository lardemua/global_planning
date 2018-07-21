# Navegação global

Navegação global do ATLASCAR2.

## Pré requisitos

    Base de dados local Postgresql.
    *API key*  - *Google Maps API Directions*



# Utilização 

Ligação internet.


```
<?xml version="1.0"?>
<launch>
    <!-- Needed only for searching for services -->
    <node pkg="global_planning" name="global_planning_server" type="global_planning_server" output="screen" />
        <param name="api_key_google_maps_api_directions" value="************************************"  />

    <node pkg="global_planning" name="global_planning_client" type="global_planning_client" output="screen"/>
     <!-- parameters to local database -->
        <param name="dbname" value="atlas_navegacao_global"  />
        <param name="user" value="pedro"  />
        <param name="password" value="*****"  />
        <param name="hostaddr" value="127.0.0.1"  />  
        <param name="port" value="5432"  />  
</launch>
```


```
roslaunch global_planning global_planning.launch
```



## Teste

No terminal verificar se o serviço (API *key*) está operacional
```
curl "https://maps.googleapis.com/maps/api/directions/json?origin=40.63754366770112,-8.658705257841461&destination=40.63567760500776,-8.654912614294403&key=*****"
```

```
{
   "geocoded_waypoints" : [
      {
         "geocoder_status" : "OK",
         "place_id" : "ChIJ8eNlvwGYIw0RBA6yPAjGOB8",
         "types" : [ "street_address" ]
      },
      {
         "geocoder_status" : "OK",
         "place_id" : "ChIJw4TTLQeYIw0RiCWRkIbtO18",
         "types" : [ "route" ]
      }
   ],
   "routes" : [
      {
         "bounds" : {
            "northeast" : {
               "lat" : 40.6378838,
               "lng" : -8.654959099999999
            },
            "southwest" : {
               "lat" : 40.6351735,
               "lng" : -8.6587364
            }
         },
         "copyrights" : "Map data ©2018 Google",
         "legs" : [
            {
               "distance" : {
                  "text" : "0.6 km",
                  "value" : 555
               },
               "duration" : {
                  "text" : "1 min",
                  "value" : 88
               },
               "end_address" : "Av. Artur Ravara, 3810-164 Aveiro, Portugal",
               "end_location" : {
                  "lat" : 40.6357075,
                  "lng" : -8.654959099999999
               },
               "start_address" : "R. da Pega 19, 3810-164 Aveiro, Portugal",
               "start_location" : {
                  "lat" : 40.6375552,
                  "lng" : -8.6587364
               },
               "steps" : [
                  {
                     "distance" : {
                        "text" : "41 m",
                        "value" : 41
                     },
                     "duration" : {
                        "text" : "1 min",
                        "value" : 4
                     },
                     "end_location" : {
                        "lat" : 40.6378838,
                        "lng" : -8.6585249
                     },
                     "html_instructions" : "Head \u003cb\u003enortheast\u003c/b\u003e on \u003cb\u003eR. da Pega\u003c/b\u003e toward \u003cb\u003eRua de Calouste Gulbenkian\u003c/b\u003e/\u003cb\u003eN235\u003c/b\u003e",
                     "polyline" : {
                        "points" : "w_`wFbdzs@_Ak@"
                     },
                     "start_location" : {
                        "lat" : 40.6375552,
                        "lng" : -8.6587364
                     },
                     "travel_mode" : "DRIVING"
                  },
                  {
                     "distance" : {
                        "text" : "0.3 km",
                        "value" : 345
                     },
                     "duration" : {
                        "text" : "1 min",
                        "value" : 47
                     },
                     "end_location" : {
                        "lat" : 40.6357095,
                        "lng" : -8.655678999999999
                     },
                     "html_instructions" : "Turn \u003cb\u003eright\u003c/b\u003e onto \u003cb\u003eRua de Calouste Gulbenkian\u003c/b\u003e/\u003cb\u003eN235\u003c/b\u003e",
                     "maneuver" : "turn-right",
                     "polyline" : {
                        "points" : "wa`wFvbzs@FUFQFOBId@_Af@q@pC_EVYZg@RYFI\\c@Ve@JEb@M"
                     },
                     "start_location" : {
                        "lat" : 40.6378838,
                        "lng" : -8.6585249
                     },
                     "travel_mode" : "DRIVING"
                  },
                  {
                     "distance" : {
                        "text" : "0.2 km",
                        "value" : 169
                     },
                     "duration" : {
                        "text" : "1 min",
                        "value" : 37
                     },
                     "end_location" : {
                        "lat" : 40.6357075,
                        "lng" : -8.654959099999999
                     },
                     "html_instructions" : "At the roundabout, take the \u003cb\u003e3rd\u003c/b\u003e exit onto \u003cb\u003eAv. Artur Ravara\u003c/b\u003e\u003cdiv style=\"font-size:0.9em\"\u003eDestination will be on the right\u003c/div\u003e",
                     "maneuver" : "roundabout-right",
                     "polyline" : {
                        "points" : "et_wF~pys@DHDFFFHFJFHDLDBAB?BABABA@ABCBC@A?A@A?A@??A?A@E@E?E@E?C?EAE?EAGAGCGCECEEEECECEAIAa@k@UW"
                     },
                     "start_location" : {
                        "lat" : 40.6357095,
                        "lng" : -8.655678999999999
                     },
                     "travel_mode" : "DRIVING"
                  }
               ],
               "traffic_speed_entry" : [],
               "via_waypoint" : []
            }
         ],
         "overview_polyline" : {
            "points" : "w_`wFbdzs@_Ak@FUNa@h@iAxDqFnAeBt@iAn@SJPPNTLPBNEHIFQBUCYM[WOIAa@k@UW"
         },
         "summary" : "Rua de Calouste Gulbenkian/N235",
         "warnings" : [],
         "waypoint_order" : []
      }
   ],
   "status" : "OK"
}

```

## Pedido de rota

ver README do package MAPVIZ


## Subscreve 
/bestpos     ->posição global.

/inspva      -> orientação do veículo.

/destination   -> Destino da missão, introduzido na aplicação Mapviz.

## Publica  

/waypoints_full  ->Lista de waypoints obtidas a partir da descodificação da polyline.

/waypoints_steps ->Lista de etapas. (Pontos que têm associado uma instrução de navegação).

/waypoints_previous_next -> pontos que definem o pedaço de caminho, utilizado pelo planeador de trajetórias , valores em metros no referencial do carro.

/waypoints_previous_next_wsg84  ->    pontos que definem o pedaço de caminho, utilizado pela aplicação MAPVIZ, valores no referencial do wsg84.

# Instalação

Através do github ATLASCAR.


## Autor
* **Pedro Bouça Nova** - *Dissertação de Mestrado - 2018* -




