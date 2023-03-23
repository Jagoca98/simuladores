from gymnasium.envs.registration import register

register(
    id="gymnasium_csv-v0",
    entry_point="gymnasium_csv.envs:GridWorldEnv",
)
