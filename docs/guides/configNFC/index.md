# Configure cuplTag with NFC

[Configuration strings](https://github.com/cuplsensor/cupltag/blob/master/docs/specs/configstrings.rst) can be written to the the tag inside an NDEF text record. 

The tag detects the text record and will reset within one minute. Strings are read and parsed on startup.

NFC configuration allows end users to make changes in the field. Most often this will be to the sample type or interval. In rare cases, the web application will go down and the base URL will need to be modified. If the tag is conformally coated, it is difficult to make a [wired connection](https://github.com/cuplsensor/cupltag/blob/master/docs/guides/configUARTpt1/index.md) after production. 

:fire: Do you use [**Chrome for Android V97+**](https://caniuse.com/webnfc)? Watch these [instructions](https://www.youtube.com/watch?v=UxrxsgZ2lA8) for an app-free alternative.

1. Download the NFC Tools app from [Google Play](https://play.google.com/store/apps/details?id=com.wakdev.wdnfc&hl=en_GB&gl=US) or the [iPhone](https://apps.apple.com/us/app/nfc-tools/id1252962749) App Store.
2. Select the **WRITE** tab. 
   <img src="60BGxm4M3apO-wDlpLX9hm0JeXo56qskCQUO64CFIahVJQyrxaFDz9D8s3nmgdoPhKqHMeSLjlR8Pj0kqb4jUmXF-wrbx6M4Nn4Q8aLvbZuTppOnV70UvAkOamZz11Bc80na4s9_FuDLDw" alt="img" style="zoom:50%;" />
3. Select **Add a Record**.
4. Select **Text**.
5. Enter one or more configuration strings. Do not use spaces, carriage returns or separators between strings. 
   *The example below sets the time interval between samples to 5 minutes.*
   <img src="rLmx2GePm_cUhRyNkNNtezwACFNb9Ia8IyLgGDjWLMv-eVh9TZ6nADZBLOTky_TMXlIl3b0jHLCiTefZ6h1uZDI7dSdc-H4VXUbsJsftBAdGQI4lYTxvTMCj_YIRRJUEDLZ2ZEBo81_bxw" alt="img" style="zoom:50%;" />
6. Press **OK**.
7. The next screen shows the new NDEF text record inside an NDEF message. 
   <img src="hvBnW_4P8HtN7nY65B4PI5OydrXSGDFTrkvzWQY-WLAmIBlX1KdSmtqo6zWajSjL0xInTYzL_Nt81cHyZMuFvWT-u0ekRevFzyXAgGCt5ZYK57O2GEWpgHlrapnq-MJBw_dZKL3INWkuYg" alt="img" style="zoom:50%;" />
8. Select **Write**.
   <img src="7Dnvg7BMVjpIttjtIqDv22Ox50CltXmHXalORePwBse6HE1Z4Lpnsh8L1UEd4B_1E0DgQn2JG_ArvOFowb-4BCC389s1SUb4C-LkmOxa_XukOegaobVET1YhfTpnWYiOyDYXaPHaSpUTHw" alt="img" style="zoom: 50%;" />
9. Tap to write the NDEF message.
   <img src="IhDI4u3RQs288cYjPfYdaCx3VzyvvbOGrwF4fOXbU7njRAw2P9ONpSVbPGHWOC6cnzGDrYR3FoXtsMI_oN8fJrjWsxbeqjfUJVszRyP8dpxFl6j_oeJjYVbS1OKGvm2nTKZ-7EtLLTms3w" alt="img" style="zoom:50%;" />
10. Wait for one minute or press the RESET button. 
    <img src="image-20220203152942897.png" alt="image-20220203152942897" style="zoom: 67%;" />





