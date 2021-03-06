function [rho, mach] = standardAtmosphereLookup(alt, aspd)

    table = [...
                         0          1.22522568276177          340.262648552556
                       200          1.20186568722271           339.49425087756
                       400          1.17884765023669          338.724158444018
                       600          1.15616810316841          337.952359661827
                       800          1.13382359660143          337.178842808977
                      1000          1.11181070032557          336.403596029441
                      1200          1.09012600332414          335.626607331023
                      1400          1.06876611376104          334.847864583158
                      1600          1.04772765896782          334.067355514671
                      1800          1.02700728543046          333.285067711482
                      2000          1.00660165877616           332.50098861427
                      2200         0.986507463759908          331.715105516085
                      2400         0.966721404250966          330.927405559907
                      2600         0.947240203219177          330.137875736154
                      2800          0.92806060272117           329.34650288014
                      3000         0.909179363886425          328.553273669474
                      3200         0.890593266903188          327.758174621403
                      3400         0.872299111004254          326.961192090101
                      3600         0.854293714452611          326.162312263896
                      3800         0.836573914526946          325.361521162433
                      4000         0.819136567507005          324.558804633781
                      4200         0.801978548658812          323.754148351471
                      4400          0.78509675221974           322.94753781147
                      4600         0.768488091383451          322.138958329087
                      4800         0.752149498284673          321.328395035809
                      5000         0.736077923983836          320.515832876063
                      5200         0.720270338451563          319.701256603911
                      5400         0.704723730553016          318.884650779662
                      5600         0.689435108032072          318.065999766411
                      5800         0.674401497495367          317.245287726496
                      6000         0.659619944396179          316.422498617875
                      6200         0.645087513018158          315.597616190412
                      6400         0.630801286458896          314.770623982087
                      6600         0.616758366613344          313.941505315107
                      6800         0.602955874157073          313.110243291926
                      7000         0.589390948529372          312.276820791178
                      7200         0.576060747916183          311.441220463503
                      7400         0.562962449232879          310.603424727277
                      7600         0.550093248106879          309.763415764238
                      7800         0.537450358860088          308.921175515009
                      8000         0.525031014491183          308.076685674501
                      8200         0.512832466657727          307.229927687219
                      8400         0.500851985658109          306.380882742435
                      8600         0.489086860413321          305.529531769254
                      8800         0.477534398448554          304.675855431549
                      9000         0.466191925874631          303.819834122771
                      9200         0.455056787369259          302.961447960629
                      9400         0.444126346158099          302.100676781632
                      9600         0.433397983995669          301.237500135496
                      9800         0.422869101146059          300.371897279397
                     10000         0.412537116363468           299.50384717209
                     10200         0.402399466872558          298.633328467864
                     10400         0.392453608348618          297.760319510337
                     10600         0.382697014897552          296.884798326103
                     10800         0.373127179035668          296.006742618191
                     11000         0.363741611669282          295.126129759362
                     11200         0.352646778965187          295.042318998479
                     11400         0.341695930393102          295.042318998479
                     11600         0.331085141879981          295.042318998479
                     11800          0.32080385343653          295.042318998479
                     12000         0.310841832996038          295.042318998479
                     12200         0.301189166231301          295.042318998479
                     12400         0.291836246687756          295.042318998479
                     12600         0.282773766223022          295.042318998479
                     12800         0.273992705743317          295.042318998479
                     13000         0.265484326227543          295.042318998479
                     13200         0.257240160030106          295.042318998479
                     13400         0.249252002453805          295.042318998479
                     13600          0.24151190358442          295.042318998479
                     13800         0.234012160378854          295.042318998479
                     14000         0.226745308998969          295.042318998479
                     14200         0.219704117383481          295.042318998479
                     14400         0.212881578050525          295.042318998479
                     14600         0.206270901123718          295.042318998479
                     14800         0.199865507574792          295.042318998479
                     15000         0.193659022676058          295.042318998479
                     15200         0.187645269656205          295.042318998479
                     15400         0.181818263553092          295.042318998479
                     15600         0.176172205257445          295.042318998479
                     15800         0.170701475741509          295.042318998479
                     16000          0.16540063046692          295.042318998479
                     16200         0.160264393966233          295.042318998479
                     16400         0.155287654592712          295.042318998479
                     16600         0.150465459433156          295.042318998479
                     16800         0.145793009378694          295.042318998479
                     17000         0.141265654348656          295.042318998479
                     17200         0.136878888662753          295.042318998479
                     17400         0.132628346556968          295.042318998479
                     17600         0.128509797838692          295.042318998479
                     17800         0.124519143676784          295.042318998479
                     18000         0.120652412522365          295.042318998479
                     18200          0.11690575615628          295.042318998479
                     18400         0.113275445859305          295.042318998479
                     18600         0.109757868701275          295.042318998479
                     18800         0.106349523945456          295.042318998479
                     19000         0.103047019564564          295.042318998479
                     19200        0.0998470688649783          295.042318998479
                     19400         0.096746487215784          295.042318998479
                     19600        0.0937421888793859          295.042318998479
                     19800        0.0908311839405451          295.042318998479
                     20000        0.0880105753307772          295.042318998479
                     20200        0.0852245519544603           295.13506320305
                     20400         0.082505249072296          295.270310768972
                     20600        0.0798750881999615          295.405487964543
                     20800        0.0773310692780488          295.540594887115
                     21000        0.0748702958340499          295.675631633812
                     21200        0.0724899713152316          295.810598301529
                     21400         0.070187395554531          295.945494986936
                     21600        0.0679599613645342          296.080321786478
                     21800        0.0658051512547799          296.215078796372
                     22000        0.0637205342678189          296.349766112612
                     22200        0.0617037629296285          296.484383830969
                     22400        0.0597525703101486           296.61893204699
                     22600        0.0578647671898695             296.753410856
                     22800        0.0560382393285504          296.887820353102
                     23000        0.0542709448323015          297.022160633179
                     23200        0.0525609116153997          297.156431790894
                     23400        0.0509062349533498          297.290633920689
                     23600        0.0493050751238293          297.424767116789
                     23800        0.0477556551322873          297.558831473202
                     24000        0.0462562585190846          297.692827083715
                     24200        0.0448052272451828          297.826754041902
                     24400        0.0434009596534998          297.960612441121
                     24600        0.0420419085031596          298.094402374511
                     24800        0.0407265790739667          298.228123935002
                     25000         0.039453527338537          298.361777215307
                     25200        0.0382213581996108          298.495362307926
                     25400        0.0370287237901692          298.628879305148
                     25600        0.0358743218340615          298.762328299049
                     25800        0.0347568940649375          298.895709381495
                     26000        0.0336752247013625          299.029022644141
                     26200        0.0326281389760697          299.162268178433
                     26400        0.0316145017173814          299.295446075607
                     26600        0.0306332159809048          299.428556426691
                     26800        0.0296832217296774          299.561599322507
                     27000        0.0287634945610061          299.694574853669
                     27200        0.0278730444783063          299.827483110583
                     27400         0.027010914706315          299.960324183452
                     27600        0.0261761805481068          300.093098162272
                     27800        0.0253679482824038          300.225805136836
                     28000        0.0245853540997248          300.358445196733
                     28200        0.0238275630759717          300.491018431349
                     28400        0.0230937681821058          300.623524929868
                     28600        0.0223831893286128           300.75596478127
                     28800        0.0216950724435062          300.888338074337
                     29000        0.0210286885826631          301.020644897649
                     29200        0.0203833330713313          301.152885339586
                     29400        0.0197583246756903           301.28505948833
                     29600        0.0191530048033865          301.417167431864
                     29800         0.018566736732008          301.549209257971
                     30000         0.017998904864496           301.68118505424
        ];
    
    rho = interp1(table(:,1), table(:,2), alt);
    mach = aspd / interp1(table(:,1), table(:,3), alt);
    
    
end