# Suivi de piste — mission 0 du TP

Le package reste nommé `urmc_mission_01`. Son contrôleur complet, la recherche de piste et le détour local sont dans `urmc_mission_01/logic.py` ; ils utilisent le socle partagé et des capteurs complets.

Le contrôleur caméra testé sur le robot est `kit_robot/suivi_piste_camera.py`. Il se lance en une commande, adapte le seuil du blanc à l’éclairage et effectue un seul essai à 5 cm/s maximum. Le menu Mac le lance par SSH. Les essais réels avec virage ont été confirmés par l’opérateur ; le damier et le STOP restent à valider séparément.

Règles : `REGLES.md`. Paramètres du contrôleur standard : `config/mission.yaml`. Simulation et robot standard ont chacun leur lanceur dans `launch/`.
