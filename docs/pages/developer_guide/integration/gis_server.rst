Setup GIS server
______________________________________________________

GISNav requires access to a :term:`GIS` server that serves high resolution
:term:`orthoimagery <Orthoimagery>` for the approximate :term:`global position`
of the :term:`vehicle`. The orthoimagery consisting of :term:`orthophoto` and
optional :term:`DEM` rasters are requested from a :term:`WMS` service which
allows querying rasters by an arbitrary :term:`bounding box`, and DEM elevation
values by an arbitrary global position via its :term:`GetMap` requests.

The DEM is optionally used to input ground elevation z-coordinates to the
:term:`PnP` problem solved by GISNav's :term:`pose` estimation algorithm in
:class:`.PoseNode`. If a DEM is not available GISNav simply assumes a planar
ground elevation, which may be sufficient when flying at higher altitudes where
an isometric perspective does not significantly distort the perceived image.

Example setups
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You are encouraged to self-host an :term:`onboard` GIS server with public domain
orthoimagery because in a realistic scenario the GIS should be embedded onboard
and not depend on an internet connection. For development it may sometimes be
more convenient to proxy an existing commercial tile-based endpoint.

.. note::
    Commercial web-based map services are often `tile-based`_ (as opposed to WMS)
    because it is more efficient to serve pre-rendered tiles than to render unique
    rasters for each individual requested bounding box. You will need a WMS proxy
    if you decide to go with a tile-based endpoint.

    .. _tile-based: https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames

.. warning::
    Many commercial services explicitly prohibit the caching of map tiles in
    their Terms of Use (ToU), especially if their business model is based on
    billing API requests. This is mainly to prevent disintermediation in case
    their tiles are redistributed to a large number of end users.

    While caching tiles onboard your own drone is likely not the kind of misuse
    targeted by such clauses, you should still make sure you understand the ToU
    of the service you are using and that it fits your planned use case.

If you are fine with using maps for the :term:`KSQL` airport area only, then you
can use the :ref:`provided Docker Compose mapserver service <Overview of services>`,
otherwise follow these instructions to self-host a :term:`MapServer` instance:

.. seealso::
    See :ref:`GIS software` for :term:`free and open-source software (FOSS)
    <FOSS>` alternatives for MapServer

To follow these instructions you will need:

* An :term:`AWS` account and AWS CLI, **or alternatively**, an `EarthExplorer`_
  account
* :term:`GDAL` installed

.. _EarthExplorer: https://earthexplorer.usgs.gov

In this example we will download :term:`NAIP` imagery and host it using
the `MapServer docker image`_ from Docker Hub. You can download the
GeoTIFF imagery from EarthExplorer, or from the Esri-maintained `AWS S3 bucket`_
if you already have AWS CLI set up:

.. _MapServer docker image: https://hub.docker.com/r/camptocamp/mapserver
.. _AWS S3 bucket: https://registry.opendata.aws/naip/

.. warning::
    This is a **Requester Pays** bucket and the files can be very large so
    download only what you need.

.. code-block:: bash
    :caption: Download a NAIP imagery product from the AWS S3 bucket

    cd ~/gisnav-docker
    mkdir -p mapfiles/
    aws s3 cp \
      --request-payer requester \
      s3://naip-source/ca/2020/60cm/rgbir_cog/37122/m_3712230_se_10_060_20200524.tif \
      mapfiles/

.. note::
    NAIP imagery is in the public domain. However, you must create an EROS
    account to download the rasters from EarthExplorer, or use secondary sources
    such as the AWS S3 bucket mentioned above.

    You do not need an account to browse for product IDs with EarthExplorer.
    An account is only needed if you want to download products.

Once you have the imagery, use GDAL to make a ``naip.vrt`` VRT file out of your
downloaded GeoTIFFs:

.. code-block:: bash
    :caption: Use GDAL to create a VRT from TIFF files

    cd mapfiles/
    gdalbuildvrt naip.vrt *.tif

Once you have your .tif and .vrt files, you can run host them through a MapServer
container:

.. code-block:: bash
    :caption: Serve the map layer using the MapServer Docker image

    export MAPSERVER_PATH=/etc/mapserver
    docker run \
      -p 80:80 \
      -v $PWD/mapfiles/:$MAPSERVER_PATH/:ro \
      camptocamp/mapserver

Test your MapServer WMS service by opening the capabilities XML in your browser:

.. code-block:: bash
    :caption: Launch a WMS GetCapabilities request in Firefox

    firefox "http://localhost:80/?map=/etc/mapserver/wms.map&service=WMS&request=GetCapabilities"

GIS software
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you want to run your own GIS server or WMS proxy, you may want to consider
e.g. these :term:`FOSS` options:

* :term:`MapServer`

* `GeoServer`_ (full-fledged OGC-compliant GIS server)

* `Mapnik`_ and `MapProxy`_

.. _GeoServer: https://geoserver.org
.. _Mapnik: https://mapnik.org
.. _MapProxy: https://mapproxy.org

Orthoimagery and DEMs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you do not want to use commercial (=not free) high-resolution imagery, various
national agencies often provide country-specific aerial imagery in the public
domain or with public-domain-like licensing terms. You should look for imagery
available in :term:`GDAL` supported formats with coverage for your flight mission
region. These may be provided as downloadable products or through
:term:`OGC`-compliant web services such as :term:`WMS` or :term:`WMTS`.

Below are just a few examples of national agencies providing high-resolution
orthoimagery that should be suitable for use with GISNav:

* `USGS High Resolution Orthoimagery`_ (USA)
* `Environment Agency Vertical Aerial Photography`_ (United Kingdom)
* `NLS orthophotos`_ (Finland)

.. _USGS High Resolution Orthoimagery: https://www.usgs.gov/centers/eros/science/usgs-eros-archive-aerial-photography-high-resolution-orthoimagery-hro
.. _Environment Agency Vertical Aerial Photography: https://www.data.gov.uk/dataset/4921f8a1-d47e-458b-873b-2a489b1c8165/vertical-aerial-photography
.. _NLS orthophotos: https://www.maanmittauslaitos.fi/en/maps-and-spatial-data/expert-users/product-descriptions/orthophotos

.. note::
    If you have a drone, you can also use readily available `photogrammetry`_
    software to create your own maps for your local region of interest.

.. _photogrammetry: https://en.wikipedia.org/wiki/Photogrammetry

Rasterizing vector data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
In some cases useful map data is not directly provided in raster but in vector
format. The GISNav ``mapserver`` service uses vector-format elevation data from
`OSM Buildings`_ to determine building heights in the simulation area to improve
accuracy* of pose estimates especially at lower flight altitudes where the
perceived planarity of the terrain is lower. For an example on how the vector
data is rasterized using GDAL, see the `mapserver service Dockerfile`_.

.. note::
    \*The GISNav SITL demo simulation does not actually benefit from the building
    height data because the simulated KSQL Airport model buildings are all
    featureless black blocks. See :ref:`SITL simulation quirks` for more
    information.

.. _OSM Buildings: https://osmbuildings.org/
.. _mapserver service Dockerfile: https://github.com/hmakelin/gisnav/blob/master/docker/mapserver/Dockerfile

SITL simulation quirks with DEMs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :term:`KSQL` :term:`Gazebo` world buildings in the SITL simulation demo are
featureless grey blocks, so any pose estimation model will most likely not use
them for matching. This means any building elevation data (see :ref:`Rasterizing
vector data`) will not technically be used to improve pose estimates in the
SITL simulation. The below figure illustrates how :term:`LoFTR` finds keypoints
at an even density throughout the simulated vehicle's field of view except on the
featureless buildings.

.. figure:: ../../../_static/img/gisnav_sitl_featureless_buildings.jpg

    LoFTR does not find keypoints on featureless buildings or terrain (SITL
    simulation)
