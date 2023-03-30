import pydantic


class BaseConfig(pydantic.BaseModel, extra=pydantic.Extra.forbid, arbitrary_types_allowed=True):
    pass
