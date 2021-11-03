# Projeto de Róbotica

Projeto de Robótica 2021.2

*Para ter acesso completo as funcionalidades deste repositório, caso não possua algumas das bibliotecas, realizar:*
> pip install -r requirements.txt

*Para poder ver a simulação basta realizar::*
> git clone https://github.com/insper-classroom/projeto-de-robotica-reverie
>
> roslaunch my_simulation trevo.launch
>
> roslaunch mybot_description mybot_control2.launch 
>
> rosrun projeto_reverie robo.py

*Integrantes*:

- Letícia Côelho
- Lorran Caetano  
- Matheus Oliveira
- Ykaro Andrade

> Status do Projeto: Em desenvolvimento :warning: 

* Conceito I : Atingido :heavy_check_mark:
* Conceito C : Atingido :heavy_check_mark:
* Conceito C+ : Atingido :heavy_check_mark:
* Conceito B : Buscando :warning:
Para atingir o conceito B, faltam acertar todas as ID's, especificas para creepers poscionados mais lateralmente (utilizando odometria) e incluir identificação da estação via rede neural, e adição do estado de busca, e posicionamento na estação.

# Objetivos 

Cores válidas do creeper: blue, green, orange
Estações válidas: dog, horse, cow e car


# Objetivos que devem ser filmados 

```python
goal1 = ("blue", 12, "dog")

goal2 = ("green", 23, "car")

goal3 = ("orange", 11, "cow")
```
```


A lista de todas as possibilidades que seu programa pode encontrar [está neste link](./todas_possibilidades.md). Lembre-se de que o código deve estar preparado para funcionar com *qualquer uma*. 

