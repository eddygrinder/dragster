# Projeto Dragster — Festival Nacional de Robótica

## O que o projeto faz
Este repositório reúne toda a componente de **software**, **experiências** e **tutoriais** relacionados com o desenvolvimento do **projeto Dragster** para o **Festival Nacional de Robótica**.  
O objetivo é documentar e partilhar o processo de programação, testes e otimização do sistema de controlo do robô, desenvolvido com **ESP32** e **FreeRTOS** no **VS Code (ESP-IDF)**.

## Por que o projeto é útil
- Centraliza toda a parte de **software e documentação técnica** do projeto.  
- Facilita a **colaboração** entre os membros da equipa.  
- Serve de **referência educativa** para outros participantes ou interessados em aprender sobre **FreeRTOS**, **sistemas embarcados** e **controlo de robôs**.  
- Garante a **reprodutibilidade das experiências**, promovendo um desenvolvimento mais rigoroso e organizado.

## Como os utilizadores podem começar a usar o projeto
1. **Clonar o repositório:**
   ```bash
   git clone https://github.com/teu-utilizador/nome-do-repositorio.git
````

2. **Abrir no VS Code** com a extensão **ESP-IDF** instalada.
3. **Configurar o ambiente ESP32** (selecionar porta, chip e SDK).
4. **Compilar e carregar o firmware** para o ESP32:

   * Através do menu ESP-IDF: *Build*, *Flash*, *Monitor*
   * Ou via terminal:

     ```bash
     idf.py build flash monitor
     ```
5. **Consultar as pastas principais:**

   * `src/` → código principal
   * `experiencias/` → testes e medições
   * `tutoriais/` → guias de configuração e utilização
```
## Onde os utilizadores podem obter ajuda

* **Documentação oficial ESP-IDF:** [https://docs.espressif.com](https://docs.espressif.com)
* **FreeRTOS oficial:** [https://www.freertos.org](https://www.freertos.org)
* **Issues do repositório:** use a aba [Issues](../../issues) para reportar problemas ou colocar dúvidas.
* Pode também contactar os **membros da equipa** através da secção *Discussions*.

## Quem mantém e contribui com o projeto

O projeto é mantido por uma equipa dedicada ao **Festival Nacional de Robótica**, responsável pelo desenvolvimento do sistema Dragster.
As contribuições são bem-vindas — basta criar um *fork* e submeter um *pull request* com as alterações propostas.

## Equipa
Eduardo Ramalhadeiro | MArco Vasconcelos - Agrupamento de Escolas Dr. erafim Leite S.J. da Madeira

**Tecnologias principais:** ESP32 · FreeRTOS · C/C++ · VS Code · ESP-IDF
**Objetivo:** Participação no Festival Nacional de Robótica — Categoria Dragster
**Equipa:** Desenvolvimento e implementação de sistema de controlo para robô Dragster
