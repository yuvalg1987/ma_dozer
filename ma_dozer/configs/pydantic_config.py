import pydantic

class BaseConfig(pydantic.BaseModel, extra=pydantic.Extra.forbid):
    pass