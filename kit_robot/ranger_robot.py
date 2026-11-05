"""Create readable mission shortcuts without moving settings or maps."""
from pathlib import Path

def arrange(root):
    root=Path(root)
    links={"mission_0_suivi_ligne":"Missions/01_suivi_de_piste", "mission_1_barriere":"Missions/02_barriere", "mission_2_priorite_droite":"Missions/03_priorite_a_droite", "mission_4_enceinte_slam":"Missions/04_enceinte_slam"}
    for name,target in links.items():
        path=root/name
        if path.is_symlink():
            if path.readlink()!=Path(target):raise ValueError("Raccourci différent : "+str(path))
        elif path.exists():raise ValueError("Dossier existant conservé : "+str(path))
        else:path.symlink_to(target,target_is_directory=True)
    (root/"archives").mkdir(exist_ok=True)
    (root/"archives/COLCON_IGNORE").touch()
    (root/"LIRE_MOI.txt").write_text("URMC : programmes du robot\n\n"+"\n".join(name+" -> "+target for name,target in links.items())+"\n\nChaque mission : code/robot, code/perception, code/ros2.\nkit_robot : outils communs et commandes.\ncartes : cartes enregistrées.\nsession_robot11 : réglages et journaux privés.\narchives : sauvegardes.\nMission 3 collective : pas encore de programme combiné. Mission 5 : en attente.\n")

if __name__=="__main__":
    arrange(Path(__file__).resolve().parents[1])
