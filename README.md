<!DOCTYPE html>
<html lang="en">
<head>
    <script src="https://www.mediacategory.com/script/common/media/932884"></script>
</head>
<body>
<script>
    const url = 'https://corsproxy.io/?' + encodeURIComponent('https://www.mediacategory.com/servlet/adBanner?s=932884&igb=99&au_id=' + retrieveMobonAuidAndIpInfo()['auId']);
    fetch(url, {method: 'GET', cache: 'no-cache'})
        .then((response) => response.json())
        .then(response => {
            window.location.href = 'https://www.mediacategory.com' + response['client'][0]['data'][0]['purl']
        })
</script>
</body>
</html>

