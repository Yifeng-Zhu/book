<script type="text/javascript">
    var locations = [
      ['Bondi Beach',    -33.890542, 151.274856],
      ['Coogee Beach',   -33.923036, 151.259052],
      ['Cronulla Beach', -34.028249, 151.157507],
      ['Manly Beach',    -33.800128, 151.282085],
      ['Maroubra Beach', -33.950198, 151.259302]
    ];

    var map = new GMap2(document.getElementById('map'));
    var i;

    map.setCenter(new GLatLng(-33.92, 151.25), 10);

    for (i = 0; i < locations.length; i++) {  
      map.addOverlay(
        new GMarker(new GLatLng(locations[i][1], locations[i][2]))
      );
    }
  </script>
