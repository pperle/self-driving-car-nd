import os
from io import BytesIO
from urllib.request import urlopen
from zipfile import ZipFile

zipurl = 'https://d17h27t6h515a5.cloudfront.net/topher/2017/February/5898cd6f_traffic-signs-data/traffic-signs-data.zip'
with urlopen(zipurl) as zipresp:
    with ZipFile(BytesIO(zipresp.read())) as zfile:
        zfile.extractall(os.path.dirname(os.path.realpath(__file__)) + '/data')
