# Mission 2 — Barrière bleue

Le rectangle bleu est horizontal quand la barrière est fermée et vertical quand elle est levée. La perception partagée dans `urmc_core/barrier.py` utilise les moments centrés d’ordre 2 pour estimer son orientation. Les règles officielles sont dans `REGLES.md` et `regles.json` ; les documents représentent une barre rouge/blanche, le bleu correspond au matériel d’essai.

## Script utilisé sur le robot

Depuis la racine du dépôt sur le Mac : `python3 tools/mac_robot.py barrier-run`. Le programme distant est `kit_robot/mission_barriere.py --run`, maximum 5 cm/s, sans limite de durée par défaut. Il suit les deux bordures, s’arrête devant la barrière et reprend après confirmation de l’ouverture. Il continue après le passage, jusqu’au damier ou au STOP. Les couleurs des bordures peuvent être inversées.

Une piste perdue, un écart excessif ou un panneau ambigu provoquent une pause. La reprise de piste exige trois images distinctes valides. Une disparition stable du panneau peut confirmer l’ouverture uniquement après fermeture et début d’ouverture observés ; ce comportement expérimental doit être vérifié sur le montage réel.

Sans `--run`, le script observe sans commander les moteurs. Vidéo permanente : http://127.0.0.1:8080/ après lancement via le Mac. L’enchaînement avec moteurs a été observé ; les correctifs de robustesse restent à confirmer sur un parcours complet. Pas d’évitement LiDAR dans ce script.

## Package standard

`urmc_mission_02/logic.py` conserve une fermeture observée, une ouverture visible stable et les protections LiDAR du socle. Son scénario s’arrête après un passage de 55 cm. Il reste distinct du script expérimental ci-dessus.
