GIS server
______________________________________________________
Overview
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
GISNav requires access to a GIS server that serves high resolution `orthoimagery`_ and optional
`digital elevation models`_ (DEM) for the approximate location of the aircraft. The orthoimage and DEM `rasters`_ are
requested from an `OGC Web Map Service`_ (WMS) which allows querying map images by an arbitrary `bounding box`_ via
its `GetMap`_ operation.

.. _orthoimagery: https://en.wikipedia.org/wiki/Orthophoto
.. _digital elevation models: https://en.wikipedia.org/wiki/Digital_elevation_model
.. _rasters: https://carto.com/blog/raster-vs-vector-whats-the-difference-which-is-best
.. _OGC Web Map Service: https://www.ogc.org/standards/wms
.. _bounding box: https://wiki.openstreetmap.org/wiki/Bounding_Box
.. _GetMap: https://opengeospatial.github.io/e-learning/wms/text/operations.html#getmap

.. seealso::
    The DEM is optionally used to input terrain z-coordinates (depth, or altitude) to the `PnP`_ problem solved by
    the current pose estimation algorithm. If a DEM is not available GISNav simply assumes a planar terrain which
    may be less accurate. See :ref:`Pose Estimators` for more information on how GISNav estimates aircraft pose from
    the map rasters.

    .. _PnP: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html

Example setups
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You are encouraged to self-host a GIS server with public domain :ref:`Orthoimagery and DEMs`, although it may be more
convenient to proxy an existing commercial tile-based endpoint.

.. tab-set::

    .. tab-item:: Self-hosted GIS
        :selected:

        The primary benefit of a self-hosted WMS service is that you can embed it onboard the drone and not rely on an
        internet connection. Here we provide an example on how to host your own maps using `MapServer`_. If you are
        fine with using maps for the SITL simulation demo area only, then you can simply use the Docker images from the
        `gisnav-docker`_ repository. Otherwise see the instructions below.

        .. _MapServer: https://mapserver.org/
        .. _gisnav-docker: https://github.com/hmakelin/gisnav-docker

        .. seealso::
            See :ref:`GIS software` for `free and open-source software`_ (FOSS) MapServer alternatives

        .. _free and open-source software: https://en.wikipedia.org/wiki/Free_and_open-source_software

        To follow these instructions you will need:

        * An AWS account and AWS CLI, **or alternatively**, an `EarthExplorer`_ account
        * `GDAL`_ installed

        .. _EarthExplorer: https://earthexplorer.usgs.gov
        .. _GDAL: https://gdal.org

        In this example we will download `NAIP`_ imagery and host it using the `MapServer docker image`_ from Docker
        Hub. You can download the GeoTIFF imagery from EarthExplorer, or from the Esri-maintained `AWS S3 bucket`_ if
        you already have AWS CLI set up:

        .. _NAIP: https://www.usgs.gov/centers/eros/science/usgs-eros-archive-aerial-photography-national-agriculture-imagery-program-naip
        .. _MapServer docker image: https://hub.docker.com/r/camptocamp/mapserver
        .. _AWS S3 bucket: https://registry.opendata.aws/naip/

        .. warning::
            This is a **Requester Pays** bucket and the files can be very large so download only what you need.

        .. code-block:: bash
            :caption: Example: Downloading a NAIP imagery product from the AWS S3 bucket

            cd ~/gisnav-docker
            mkdir -p mapfiles/
            aws s3 cp \
              --request-payer requester \
              s3://naip-source/ca/2020/60cm/rgbir_cog/37122/m_3712230_se_10_060_20200524.tif \
              mapfiles/

        .. note::
            * The NAIP imagery is in the public domain. However, you must create an EROS account to download
              the rasters from EarthExplorer, or use secondary sources such as the AWS S3 bucket mentioned above. The
              data is not redistributed in the `gisnav-docker`_ repository to keep its size manageable.
            * You do not need an account to browse for product IDs with EarthExplorer. An account is only needed if you
              want to download products.

        Once you have the imagery, use GDAL to make a ``naip.vrt`` VRT file out of your downloaded GeoTIFFs:

        .. code-block:: bash
            :caption: Use GDAL to create a VRT from TIFF files

            cd mapfiles/
            gdalbuildvrt naip.vrt *.tif

        Once you have your .tif and .vrt files, you can run host them through a MapServer container:

        .. code-block:: bash
            :caption: Serve the map layer using the MapServer Docker image

            cd ~/gisnav-docker
            export CONTAINER_NAME=gisnav-mapserver
            export MAPSERVER_PATH=/etc/mapserver
            docker run \
              --name $CONTAINER_NAME \
              -p 80:80 \
              -v $PWD/mapfiles/:$MAPSERVER_PATH/:ro \
              camptocamp/mapserver

        Test your MapServer WMS service by opening the capabilities XML in your browser:

        .. code-block:: bash
            :caption: Launch a WMS GetCapabilities request in Firefox

            firefox "http://localhost:80/?map=/etc/mapserver/wms.map&service=WMS&request=GetCapabilities"

    .. tab-item:: WMS proxy

        If you already have a third party high-resolution aerial or satellite imagery endpoint available, you only need
        to proxy it through a WMS service. You can follow the `gisnav-docker README.md`_ to set up a WMS MapProxy using
        the provided Docker image.

        .. _gisnav-docker README.md: https://github.com/hmakelin/gisnav-docker

        .. note::
            Commercial web-based map services are often `tile-based`_ (as opposed to WMS) because it is more
            efficient to serve pre-rendered tiles than to render unique rasters for each individual requested bounding
            box. You will need a WMS proxy if you decide to go with a tile-based endpoint.

        .. _tile-based: https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames

        .. warning::
            Many commercial services explicitly prohibit the caching of map tiles in their Terms of Use (ToU),
            especially if their business model is based on billing API requests. This is mainly to prevent
            disintermediation in case their tiles are redistributed to a large number of end users.

            While caching tiles onboard your own drone is likely not the kind of misuse targeted by such clauses, you
            should still make sure you understand the ToU of the service you are using and that it fits your planned
            use case.

GIS software
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you want to run your own GIS server or WMS proxy, you may want to consider e.g. these
`free and open-source software`_ (FOSS) options:

    * `MapServer`_

    * `GeoServer`_ (full-fledged OGC-compliant GIS server)

    * `Mapnik`_ and `MapProxy`_

.. _free and open-source software: https://en.wikipedia.org/wiki/Free_and_open-source_software
.. _GeoServer: https://geoserver.org
.. _Mapnik: https://mapnik.org
.. _MapProxy: https://mapproxy.org

Orthoimagery and DEMs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you do not want to use commercial (=not free) high-resolution imagery, various national agencies often provide
country-specific aerial imagery in the public domain or with public-domain-like licensing terms. You should look for
imagery available in `GDAL`_ supported formats with coverage for your area. These may be provided as
downloadable products or through OGC-compliant web services such as WMS or WMTS. Below are just a few examples of
national agencies providing high-resolution orthoimagery that should be suitable for use with GISNav:

* `USGS High Resolution Orthoimagery`_ (USA)
* `Environment Agency Vertical Aerial Photography`_ (United Kingdom)
* `NLS orthophotos`_ (Finland)

.. _USGS High Resolution Orthoimagery: https://www.usgs.gov/centers/eros/science/usgs-eros-archive-aerial-photography-high-resolution-orthoimagery-hro
.. _Environment Agency Vertical Aerial Photography: https://www.data.gov.uk/dataset/4921f8a1-d47e-458b-873b-2a489b1c8165/vertical-aerial-photography
.. _NLS orthophotos: https://www.maanmittauslaitos.fi/en/maps-and-spatial-data/expert-users/product-descriptions/orthophotos

.. note::
    If you have a drone, you can also use readily available `photogrammetry`_ software to create your own maps for your
    local region of interest

.. _photogrammetry: https://en.wikipedia.org/wiki/Photogrammetry

Rasterizing vector data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
In some cases useful map data is not directly provided in raster but in vector format. The GISNav SITL service in
the `gisnav-docker`_ repository utilizes vector-format elevation data from `OSM Buildings`_ to determine building
heights in the simulation area to improve accuracy* of pose estimates especially at lower flight altitudes where the
perceived planarity of the terrain is lower. For an example on how the vector data is rasterized using GDAL,
see `this gisnav-docker setup script`_.

.. note::
    \*The GISNav SITL demo simulation does not actually benefit from the building height data because the simulated
    KSQL Airport model buildings are all featureless black blocks. See :ref:`SITL simulation quirks` for more
    information.

.. _OSM Buildings: https://osmbuildings.org/
.. _this gisnav-docker setup script: https://github.com/hmakelin/gisnav-docker/blob/master/scripts/setup_mapserver.sh