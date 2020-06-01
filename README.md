# Nine Men's Morris AI 
#### Instalacija pygame biblioteke ####
Potrebno je instalirati [pygame](https://pypi.org/project/pygame/) biblioteku. 
```bash 
pip install pygame 
``` 
#### Pokretanje samo minimax algoritma s alfa-beta podrezivanjem ####
GUI se pokreće automatski pokretanjem skripte: 

```bash 
python main.py
```

#### Franka ROS/Gazebo ####
Koraci za pokretanje Gazebo svijeta s Frankom i minimax algoritmom:
1. ```bash
	roscore
   ```
2. ```bash
   roslaunch panda_sim game_world.launch
   ```
3. ```bash
   rosrun panda_sim panda_interface.py
   ```
4. ```bash
   rosrun panda_sim main.py
   ```


